# Lab 4: PWM Audio / Addressable LEDs — Code Review & Debug Notes

This document covers suspected bugs and risk areas **in the code itself**.
It assumes the Arduino toolchain and flashing are working correctly.

## Summary of Peripheral State

| Peripheral | Believed Status | Confidence |
|---|---|---|
| LEDC (PWM audio on GPIO14) | Sample playback loop advancing | High — confirmed via `LEDC_INT_RAW=0x00000011` and sample counter climbing at sample rate |
| LEDC → speaker signal path | Unknown | Zero — never tested with amp powered |
| RMT (WS2812 on GPIO33) | Broken — TX_END never fires | High — every `rmt_send_color()` call hits its timeout |
| LED visualization logic | Untested (blocked on RMT) | — |

## RMT — Primary Suspect

The RMT transmit sequence writes 24 bits into RMT RAM, resets the read
pointer, commits config, sets TX_START, and polls `RMT_CH0_TX_END_INT_RAW`.
The bit never becomes set. Possible root causes, ranked:

### 1. Wrong bit positions in `RMT_CHnCONF0_REG` on ESP32-S3

The lab PDF's Figure 4 shows the CONF0 layout for the S3, but several
fields overlap with locations that are different on the original ESP32.
Most suspect:
- `RMT_MEM_SIZE_CH0` — I have it at bit shift 16. If it's actually at a
  different position, we'd be writing to reserved bits and the channel
  would have zero memory blocks allocated, meaning nothing to transmit.
- `RMT_DIV_CNT_CH0` — I have it at bit shift 8. Same concern.
- `RMT_CARRIER_EN_CH0` — I have it at bit 20. If this is actually set
  somewhere else by default, carrier modulation would corrupt the output.
- `RMT_IDLE_OUT_EN_CH0` / `RMT_IDLE_OUT_LV_CH0` — bits 5, 6.
- `RMT_CONF_UPDATE_CH0` — bit 24.

**Debug plan:** the diagnostic code dumps `RMT_CH0_CONF0_REG` after
setup. Compare the value we wrote vs. what reads back. Any bit that
flipped or disappeared tells us that position is reserved/different.

### 2. Clock source selection in `RMT_SYS_CONF_REG`

I set `RMT_SCLK_SEL = 1` expecting that to be APB_CLK (80 MHz). The S3
TRM may number sources differently. If the RMT has no clock, nothing
transmits and TX_END never fires — which is exactly the symptom.

Values to try if 1 doesn't work: 0, 2, 3.

**Debug plan:** the diagnostic code dumps `RMT_SYS_CONF_REG`. If bits 24-25
read back as 0 despite us writing 1, the field is somewhere else.

### 3. Missing or wrong `CONF_UPDATE` sequence

The lab doc says to set `CONF_UPDATE` in CONF0 after each config change.
I set it both in `setup_RMT` (as part of the initial write) and in
`rmt_send_color` (before `TX_START`). If `CONF_UPDATE` is self-clearing
and must be pulsed, my one-shot write might not be latching the config.

**Fix to try:** explicitly write 0, then 1, then 0 to `CONF_UPDATE_CH0`
rather than just setting it once.

### 4. `MEM_RD_RST` pulse ordering

The transmit sequence does:
```
SET   MEM_RD_RST
CLEAR MEM_RD_RST
SET   CONF_UPDATE
SET   TX_START
```
If `MEM_RD_RST` on the S3 requires a minimum pulse width greater than
one APB cycle, back-to-back set/clear from two adjacent register writes
might not actually reset anything. Could cause the RMT to read stale
pointers and produce nothing.

**Fix to try:** insert a NOP or two, or read back CONF0 between the
set and clear to enforce ordering.

### 5. Clock gate on RMT RAM itself

`RMT_MEM_CLK_FORCE_ON` in `RMT_SYS_CONF_REG` is set to 1. If the bit
position is wrong, the RAM block is unclocked and writes disappear.

**Debug plan:** the diagnostic code writes `ram[0]` then prints it. If
the read returns a different value (or zero), RAM writes aren't landing.

### 6. GPIO output enable register offset

Originally had `GPIO_ENABLE1_REG` at offset `0x0024` — this was wrong.
Corrected to `0x002C`. Verify by reading back `GPIO_ENABLE1` in setup
and confirming bit 1 (for pin 33) is set.

### 7. Wrong bit polled for TX_END

`RMT_INT_RAW_REG` bit 0 is where I expect `RMT_CH0_TX_END_INT_RAW`
to be. On the S3 it might be a different bit. If the transmit is
*actually succeeding* but we're polling the wrong status bit, we'd
see exactly the timeout symptom — and the first call would timeout
while the LEDs mysteriously updated.

**Debug plan:** after the timeout fires, the diagnostic code dumps
`RMT_INT_RAW_REG`. If it's nonzero with some bit other than 0 set,
that's our real TX_END bit.

## LEDC — Secondary Suspects

LEDC appears to be running based on sample counter progression. Possible
issues that might prevent audible output even so:

### A. Wrong 10.8 fixed-point divider format

The divider computation is:
```c
double div_d = 80000000.0 / (256.0 * (double)R);
uint32_t div_whole = (uint32_t)div_d;
uint32_t div_frac  = (uint32_t)((div_d - (double)div_whole) * 256.0 + 0.5);
uint32_t div_fixed = (div_whole << 8) | div_frac;
```
This assumes whole bits in [17:8] and frac bits in [7:0] of the 18-bit
divider field. If the format is fraction-high-whole-low, all PWM periods
would be massively off and the signal would be DC-ish.

**Test:** at 16 kHz sample rate, `div_d = 80e6/(256*16000) = 19.53125`.
Whole = 19 (0x13), frac = 0.53125 * 256 = 136 (0x88). Combined = 0x1388.
Can be verified in silicon by reading back `LEDC_TIMER0_CONF_REG`.

### B. Duty cycle left-shift

`LEDC_CH0_DUTY_REG` is written as `(sample & 0xFF) << 4`. The lab doc
says "write the duty cycle into the whole bits... shift by 4 bits to
the left to set its fraction bits to 0." This looks right for an
8-bit duty value going into a duty register with 4 fraction bits.
If the duty field is wider (e.g., includes more integer bits), we'd
still get valid output but at lower amplitude.

### C. Signal output enable on channel

`LEDC_SIG_OUT_EN_CH0` (bit 2 of `LEDC_CH0_CONF0_REG`) is set. If this
isn't actually bit 2 on the S3, the channel would run internally but
never drive the pin.

**Debug plan:** scope GPIO14. If there's PWM activity, LEDC is fine
and the issue is downstream (amp power, volume pots, wiring). If
nothing, the output enable bit is wrong.

## General Code Risks

### Shift count warnings
Lines like `(1u << 31)` produce warnings on AVR but are correct on
ESP32-S3. Currently using `1u` which is 32 bits on ESP32 — fine, but
if anyone ports this to another platform, change to `1UL`.

### `build_frame` log scale
When all samples are 128 (silence), `deflection = 0`, `lit = 0`, and
the visualizer shows all LEDs dark. That's correct. For any non-silent
sample the log mapping assumes the loudest possible is full deflection
(128 away from 128), producing all 100 LEDs lit. In practice audio
rarely hits full deflection, so peak brightness in testing will be
30-60 LEDs. This is expected behavior, not a bug.

### `transmit_led_signal` spec ambiguity
The lab text says the function should update "all five LEDs" but the
colormap table in the same section has 5 entries mapped across 100
LEDs. I interpreted this as a typo for "100" and wrote the function
to iterate 100 LEDs. Worth confirming with TA if 5 full LEDs was
actually intended (unlikely given the visualization description).

### `update_PWM` polling contract
The function returns without writing when `initial == 0` and the
overflow bit is clear. The main loop calls it in a spin-poll pattern.
This is correct per the spec, but if the spec intended `update_PWM`
to *block* until the bit fires, the behavior differs. Current design
is more flexible and allows the caller to do other work between
samples (like the LED refresh).

## What to Do First

1. **Flash the current code** with the diagnostic prints enabled.
2. **Read the first ~30 lines of serial output** — these contain the
   register dumps. Compare wrote-vs-read for every RMT register. Any
   discrepancy is a bit position error in the `#define`s.
3. **Scope GPIO33** while transmitting. If you see *any* activity —
   even wrong-timing wiggles — the RMT is running and the issue is
   timing/protocol. If the line is dead flat, the RMT isn't running
   at all (clock source, MEM_SIZE, or output enable issue).
4. **Scope GPIO14** regardless of LED state. Confirms whether PWM is
   reaching the amp input. If it is but there's no audio, the issue
   is amp power or volume pots, not software.
5. **Check volume trimpots VR1, VR2, VR3** with a screwdriver before
   concluding audio is broken. These have historically been set to
   zero by previous users.

## Register Reference Cross-Check

Every `#define` in `code.ino` should be verified against the actual
ESP32-S3 Technical Reference Manual, section on RMT and LEDC
peripherals. The lab PDF simplifies some fields and the S3 differs
from the original ESP32 in ways the PDF may not fully capture.

Key registers to double-check with the S3 TRM:
- `RMT_SYS_CONF_REG` (0x600160C0) — field positions of SCLK_SEL,
  SCLK_DIV_NUM, MEM_CLK_FORCE_ON, CLK_EN
- `RMT_CHnCONF0_REG` (0x60016020 + 4*n) — field positions of DIV_CNT,
  MEM_SIZE, CARRIER_EN, IDLE_OUT_EN/LV, CONF_UPDATE, TX_START,
  MEM_RD_RST
- `RMT_INT_RAW_REG` (0x600160A0) — which bit corresponds to CH0_TX_END
- `LEDC_TIMER0_CONF_REG` (0x600190A0) — DIV field width and position
- `LEDC_CHn_CONF0_REG` — SIG_OUT_EN and PARA_UP positions
