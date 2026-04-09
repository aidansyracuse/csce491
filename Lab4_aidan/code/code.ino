// Lab 4: PWM Audio and Addressable LEDs
// CSCE 491 - Systems Engineering
//
// Plays 8-bit PCM audio out of GPIO14 via the LEDC PWM peripheral,
// and drives a 100-LED WS2812 strip on GPIO33 via the RMT peripheral
// as a log-scale audio visualizer.
//
// All peripheral setup is done by writing directly to the ESP32-S3
// hardware registers (no Arduino/IDF driver calls).

#include <Arduino.h>
#include "array.h"   // provides sampleRate and sampleArray[] (from audioToPCM.py)

// ---------- System / clock gating ----------
#define SYSTEM_PERIP_CLK_EN0_REG   0x600C0018
#define SYSTEM_PERIP_RST_EN0_REG   0x600C0020
#define SYSTEM_RMT_CLK_EN_BIT      (1u << 9)
#define SYSTEM_RMT_RST_BIT         (1u << 9)
#define SYSTEM_LEDC_CLK_EN_BIT     (1u << 11)
#define SYSTEM_LEDC_RST_BIT        (1u << 11)

// ---------- RMT registers ----------
#define RMT_BASE                   0x60016000
#define RMT_CH0_CONF0_REG          (RMT_BASE + 0x0020)    // 0x60016020
#define RMT_INT_RAW_REG            (RMT_BASE + 0x0070)    // 0x60016070
#define RMT_INT_CLR_REG            (RMT_BASE + 0x007C)    // 0x6001607C
#define RMT_SYS_CONF_REG           (RMT_BASE + 0x00C0)    // 0x600160C0
#define RMT_CH0_RAM_BASE           (RMT_BASE + 0x0800)    // 0x60016800

// RMT_SYS_CONF_REG bit positions
#define RMT_APB_FIFO_MASK_BIT      (1u << 0)
#define RMT_MEM_CLK_FORCE_ON_BIT   (1u << 1)
#define RMT_SCLK_DIV_NUM_SHIFT     4          // bits [11:4]
#define RMT_SCLK_SEL_SHIFT         24         // bits [25:24]
#define RMT_CLK_EN_BIT             (1u << 31)

// RMT_CHn_CONF0_REG bit positions
#define RMT_TX_START_CH0_BIT       (1u << 0)
#define RMT_MEM_RD_RST_CH0_BIT     (1u << 1)
#define RMT_DIV_CNT_CH0_SHIFT      8          // bits [15:8]
#define RMT_MEM_SIZE_CH0_SHIFT     16         // bits [19:16]
#define RMT_CARRIER_EN_CH0_BIT     (1u << 20)
#define RMT_CARRIER_OUT_LV_CH0_BIT (1u << 21)
#define RMT_IDLE_OUT_EN_CH0_BIT    (1u << 6)
#define RMT_IDLE_OUT_LV_CH0_BIT    (1u << 5)
#define RMT_CONF_UPDATE_CH0_BIT    (1u << 24)

// RMT_INT_*_REG: ch0 tx end is bit 0
#define RMT_CH0_TX_END_INT_BIT     (1u << 0)

// ---------- LEDC registers ----------
#define LEDC_BASE                  0x60019000
#define LEDC_CH0_CONF0_REG         (LEDC_BASE + 0x0000)
#define LEDC_CH0_DUTY_REG          (LEDC_BASE + 0x0008)
#define LEDC_CH0_CONF1_REG         (LEDC_BASE + 0x000C)
#define LEDC_TIMER0_CONF_REG       (LEDC_BASE + 0x00A0)
#define LEDC_INT_RAW_REG           (LEDC_BASE + 0x00C0)
#define LEDC_INT_CLR_REG           (LEDC_BASE + 0x00CC)
#define LEDC_CONF_REG              (LEDC_BASE + 0x00D0)

// LEDC_CONF_REG
#define LEDC_APB_CLK_SEL_BIT       (1u << 0)
#define LEDC_CLK_EN_BIT            (1u << 31)

// LEDC_TIMER0_CONF_REG
#define LEDC_TIMER0_DUTY_RES_SHIFT 0          // bits [3:0]
#define LEDC_CLK_DIV_TIMER0_SHIFT  4          // bits [21:4]  (10.8 fixed)
#define LEDC_TIMER0_PAUSE_BIT      (1u << 22)
#define LEDC_TIMER0_RST_BIT        (1u << 23)
#define LEDC_TIMER0_PARA_UP_BIT    (1u << 25)

// LEDC_CH0_CONF0_REG
#define LEDC_TIMER_SEL_CH0_SHIFT   0          // bits [1:0]
#define LEDC_SIG_OUT_EN_CH0_BIT    (1u << 2)
#define LEDC_IDLE_LV_CH0_BIT       (1u << 3)
#define LEDC_PARA_UP_CH0_BIT       (1u << 4)

// LEDC_CH0_CONF1_REG
#define LEDC_DUTY_START_CH0_BIT    (1u << 31)

// LEDC_INT_*_REG: timer0 overflow is bit 0
#define LEDC_TIMER0_OVF_INT_BIT    (1u << 0)

// ---------- GPIO matrix / IO MUX ----------
#define GPIO_BASE                  0x60004000
#define IO_MUX_BASE                0x60009000
#define GPIO_FUNC_OUT_SEL_CFG_REG(pin)  (GPIO_BASE + 0x0554 + ((pin) * 4))
#define IO_MUX_GPIO_REG(pin)            (IO_MUX_BASE + 0x0010 + ((pin) * 4))
#define IO_MUX_MCU_SEL_SHIFT            12
#define IO_MUX_FUN_IE_BIT               (1u << 9)
#define IO_MUX_FUN_DRV_SHIFT            10    // bits [11:10], drive strength

#define LEDC_FUNC_SEL   73   // LEDC ch0 -> GPIO matrix signal 73
#define RMT_FUNC_SEL    81   // RMT  ch0 -> GPIO matrix signal 81

#define PIN_LEDC  14
#define PIN_RMT   33

// ---------- LED visualization ----------
#define NUM_LEDS  100

// RMT is clocked at 80 MHz -> 12.5 ns per tick.
// WS2812 timings (rounded to ticks):
//   T0H = 400 ns  -> 32
//   T0L = 850 ns  -> 68
//   T1H = 800 ns  -> 64
//   T1L = 450 ns  -> 36
#define T0H_TICKS  32
#define T0L_TICKS  68
#define T1H_TICKS  64
#define T1L_TICKS  36

// Pack one RMT RAM entry: level0[31], period0[30:16], level1[15], period1[14:0]
static inline uint32_t rmt_entry(uint32_t lvl0, uint32_t p0, uint32_t lvl1, uint32_t p1) {
  return ((lvl0 & 1u) << 31) | ((p0 & 0x7FFF) << 16) |
         ((lvl1 & 1u) << 15) |  (p1 & 0x7FFF);
}

// One-bit entries: high-then-low pair encoded in a single RMT word.
static const uint32_t RMT_BIT_0 = (1u << 31) | (T0H_TICKS << 16) | (0u << 15) | T0L_TICKS;
static const uint32_t RMT_BIT_1 = (1u << 31) | (T1H_TICKS << 16) | (0u << 15) | T1L_TICKS;

// =====================================================================
// setup_RMT
// =====================================================================
void setup_RMT() {
  // 1. Enable RMT clock and pull out of reset
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_RMT_CLK_EN_BIT);
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_RMT_RST_BIT);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_RMT_RST_BIT);

  // 2. RMT_SYS_CONF_REG
  //    - clock source = APB (80 MHz)  => RMT_SCLK_SEL = 1
  //    - divider = 1  (NUM = 0)
  //    - enable RMT clock, force mem clock on
  //    - clear APB FIFO mask (access via APB, not FIFO)
  uint32_t sys_conf = 0;
  sys_conf |= RMT_CLK_EN_BIT;                 // bit 31
  sys_conf |= (1u << RMT_SCLK_SEL_SHIFT);     // APB_CLK
  sys_conf |= (0u << RMT_SCLK_DIV_NUM_SHIFT); // divider = NUM+1 = 1
  sys_conf |= RMT_MEM_CLK_FORCE_ON_BIT;       // bit 1
  // RMT_APB_FIFO_MASK_BIT left at 0
  REG_WRITE(RMT_SYS_CONF_REG, sys_conf);

  // 3. RMT_CH0_CONF0_REG
  //    - no carrier, DIV_CNT = 1, MEM_SIZE = 1 block, IDLE output low
  uint32_t ch0 = 0;
  ch0 |= (1u << RMT_DIV_CNT_CH0_SHIFT);   // channel divider = 1
  ch0 |= (1u << RMT_MEM_SIZE_CH0_SHIFT);  // 1 RMT RAM block
  ch0 |= RMT_IDLE_OUT_EN_CH0_BIT;         // drive idle level
  // RMT_IDLE_OUT_LV_CH0_BIT stays 0 (idle low)
  // RMT_CARRIER_EN_CH0_BIT stays 0 (no carrier)
  ch0 |= RMT_CONF_UPDATE_CH0_BIT;
  REG_WRITE(RMT_CH0_CONF0_REG, ch0);

  // 4. Clear any stale TX-end interrupt
  REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_BIT);

  // 5. Route GPIO33 to the RMT ch0 output (signal 81)
  //    IO MUX: select function mode (MCU_SEL = 2 for GPIO matrix),
  //    full drive strength, output-only.
  REG_WRITE(IO_MUX_GPIO_REG(PIN_RMT),
            (2u << IO_MUX_MCU_SEL_SHIFT) | (3u << IO_MUX_FUN_DRV_SHIFT));
  //    GPIO matrix: output select = 81, no inversion, peripheral OE.
  REG_WRITE(GPIO_FUNC_OUT_SEL_CFG_REG(PIN_RMT), RMT_FUNC_SEL);
  //    Enable output on pin 33 (bit 33-32 = bit 1 in GPIO_ENABLE1_REG).
  //    On ESP32-S3, GPIO_ENABLE1_REG lives at offset 0x002C, not 0x0024.
  REG_SET_BIT(GPIO_BASE + 0x002C, (1u << (PIN_RMT - 32)));

  // Diagnostic: dump what we actually wrote vs. what reads back.
  Serial.printf("RMT_SYS_CONF_REG   wrote=0x%08x read=0x%08x\n",
                (unsigned)sys_conf, (unsigned)REG_READ(RMT_SYS_CONF_REG));
  Serial.printf("RMT_CH0_CONF0_REG  wrote=0x%08x read=0x%08x\n",
                (unsigned)ch0, (unsigned)REG_READ(RMT_CH0_CONF0_REG));
  Serial.printf("IO_MUX_GPIO33      read=0x%08x\n",
                (unsigned)REG_READ(IO_MUX_GPIO_REG(PIN_RMT)));
  Serial.printf("GPIO_FUNC33_OUT    read=0x%08x\n",
                (unsigned)REG_READ(GPIO_FUNC_OUT_SEL_CFG_REG(PIN_RMT)));
  Serial.printf("GPIO_ENABLE1       read=0x%08x\n",
                (unsigned)REG_READ(GPIO_BASE + 0x002C));
  Serial.flush();
}

// =====================================================================
// setup_LEDC
// =====================================================================
void setup_LEDC() {
  // 1. Enable LEDC clock and pull out of reset
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_LEDC_CLK_EN_BIT);
  REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST_BIT);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST_BIT);

  // 2. LEDC_CONF_REG: enable register clock, select APB clock (80 MHz)
  REG_WRITE(LEDC_CONF_REG, LEDC_CLK_EN_BIT | LEDC_APB_CLK_SEL_BIT);

  // 3. Compute 10.8 fixed-point divider so that the 8-bit PWM period
  //    equals one audio sample period: div = 80e6 / R.
  //    (LEDC scales the timer clock by this divider, then the 8-bit
  //     counter overflows every 256 divided ticks -> one PWM period,
  //     wait: we want the PWM *frequency* to be the sample rate, so
  //     the divided clock must tick at 256 * R. Hence divider = 80e6/(256*R).)
  uint32_t R = sampleRate;
  double div_d = 80000000.0 / (256.0 * (double)R);
  uint32_t div_whole = (uint32_t)div_d;
  uint32_t div_frac  = (uint32_t)((div_d - (double)div_whole) * 256.0 + 0.5);
  if (div_frac > 255) { div_frac = 0; div_whole += 1; }
  uint32_t div_fixed = (div_whole << 8) | div_frac;   // 10.8 fixed-point

  // 4. LEDC_TIMER0_CONF_REG:
  //    DUTY_RES = 8, divider = computed, unpause, not in reset, commit.
  uint32_t timer_conf = 0;
  timer_conf |= (8u << LEDC_TIMER0_DUTY_RES_SHIFT);
  timer_conf |= (div_fixed << LEDC_CLK_DIV_TIMER0_SHIFT);
  // LEDC_TIMER0_RST_BIT left at 0 (not in reset)
  // LEDC_TIMER0_PAUSE_BIT left at 0 (not paused)
  timer_conf |= LEDC_TIMER0_PARA_UP_BIT;
  REG_WRITE(LEDC_TIMER0_CONF_REG, timer_conf);

  // 5. LEDC_CH0_CONF0_REG: select timer0, enable sig out, commit params.
  uint32_t ch_conf0 = 0;
  ch_conf0 |= (0u << LEDC_TIMER_SEL_CH0_SHIFT);  // timer 0
  ch_conf0 |= LEDC_SIG_OUT_EN_CH0_BIT;           // enable PWM output
  ch_conf0 |= LEDC_PARA_UP_CH0_BIT;              // commit
  REG_WRITE(LEDC_CH0_CONF0_REG, ch_conf0);

  // 6. Seed the duty cycle to mid-rail (silence).
  REG_WRITE(LEDC_CH0_DUTY_REG, (128u << 4));
  REG_WRITE(LEDC_CH0_CONF0_REG, ch_conf0);       // PARA_UP again
  REG_WRITE(LEDC_CH0_CONF1_REG, LEDC_DUTY_START_CH0_BIT);

  // 7. Clear any stale timer-overflow interrupt.
  REG_WRITE(LEDC_INT_CLR_REG, LEDC_TIMER0_OVF_INT_BIT);

  // 8. Route GPIO14 to the LEDC ch0 output (signal 73).
  REG_WRITE(IO_MUX_GPIO_REG(PIN_LEDC),
            (2u << IO_MUX_MCU_SEL_SHIFT) | (3u << IO_MUX_FUN_DRV_SHIFT));
  REG_WRITE(GPIO_FUNC_OUT_SEL_CFG_REG(PIN_LEDC), LEDC_FUNC_SEL);
  //    Enable output on pin 14 (GPIO_ENABLE_REG @ 0x60004020).
  REG_SET_BIT(GPIO_BASE + 0x0020, (1u << PIN_LEDC));
}

// =====================================================================
// update_PWM
//
//   initial != 0  => force-write the first sample unconditionally
//   sample        => 8-bit duty value (0..255)
//
// Returns without touching the peripheral if the previous PWM period
// has not yet completed (polled via LEDC_INT_RAW_REG).
// =====================================================================
void update_PWM(uint32_t initial, uint32_t sample) {
  if (!initial) {
    if (!(REG_READ(LEDC_INT_RAW_REG) & LEDC_TIMER0_OVF_INT_BIT)) {
      return;   // previous period still in progress
    }
    REG_WRITE(LEDC_INT_CLR_REG, LEDC_TIMER0_OVF_INT_BIT);
  }

  // Write new duty (whole bits are [18:4], so shift left 4).
  REG_WRITE(LEDC_CH0_DUTY_REG, (sample & 0xFF) << 4);
  REG_SET_BIT(LEDC_CH0_CONF0_REG, LEDC_PARA_UP_CH0_BIT);
  REG_SET_BIT(LEDC_CH0_CONF1_REG, LEDC_DUTY_START_CH0_BIT);
}

// =====================================================================
// Transmit a single 24-bit GRB color on RMT ch0 and wait for completion.
// =====================================================================
static int rmt_send_verbose = 1;  // first call prints diagnostics
static void rmt_send_color(uint32_t grb) {
  volatile uint32_t *ram = (volatile uint32_t *)RMT_CH0_RAM_BASE;

  // MSB-first: WS2812 expects G7..G0, R7..R0, B7..B0.
  for (int i = 0; i < 24; i++) {
    uint32_t bit = (grb >> (23 - i)) & 1u;
    ram[i] = bit ? RMT_BIT_1 : RMT_BIT_0;
  }
  ram[24] = 0;  // zero-termination

  if (rmt_send_verbose) {
    Serial.printf("  before start: CONF0=0x%08x INT_RAW=0x%08x ram[0]=0x%08x\n",
                  (unsigned)REG_READ(RMT_CH0_CONF0_REG),
                  (unsigned)REG_READ(RMT_INT_RAW_REG),
                  (unsigned)ram[0]);
    Serial.flush();
  }

  // Reset read pointer, commit config, start TX.
  REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_MEM_RD_RST_CH0_BIT);
  REG_CLR_BIT(RMT_CH0_CONF0_REG, RMT_MEM_RD_RST_CH0_BIT);
  REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_CONF_UPDATE_CH0_BIT);
  REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_TX_START_CH0_BIT);

  if (rmt_send_verbose) {
    Serial.printf("  after start:  CONF0=0x%08x INT_RAW=0x%08x\n",
                  (unsigned)REG_READ(RMT_CH0_CONF0_REG),
                  (unsigned)REG_READ(RMT_INT_RAW_REG));
    Serial.flush();
  }

  // Wait for TX end (~30 us for 24 bits).  Add a safety timeout so
  // a misconfigured RMT can't hang the whole sketch forever.
  uint32_t timeout = 100000;   // ~ a few ms at CPU speed
  while (!(REG_READ(RMT_INT_RAW_REG) & RMT_CH0_TX_END_INT_BIT)) {
    if (--timeout == 0) {
      if (rmt_send_verbose) {
        Serial.printf("  TIMEOUT: CONF0=0x%08x INT_RAW=0x%08x\n",
                      (unsigned)REG_READ(RMT_CH0_CONF0_REG),
                      (unsigned)REG_READ(RMT_INT_RAW_REG));
        Serial.flush();
      } else {
        Serial.println("!! rmt_send_color TIMEOUT");
        Serial.flush();
      }
      break;
    }
  }
  REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_BIT);
  rmt_send_verbose = 0;  // only first call is verbose
}

// =====================================================================
// transmit_led_signal
//
//   colors[i] holds a 24-bit value 0x00RRGGBB for LED i.
// =====================================================================
void transmit_led_signal(uint32_t *colors) {
  for (int i = 0; i < NUM_LEDS; i++) {
    uint32_t rgb = colors[i];
    uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >>  8) & 0xFF;
    uint8_t b =  rgb        & 0xFF;
    // WS2812 wants GRB order, MSB first.
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    rmt_send_color(grb);
  }
  // Latch frame: hold line low > 50 us (RMT idles low).
  delayMicroseconds(80);
}

// =====================================================================
// Visualization helpers
// =====================================================================

// Colormap for a given "lit LED count" bucket (1..5 from the lab spec),
// repeated for higher buckets.  Returned as 0x00RRGGBB.
static uint32_t color_for_level(int level) {
  // level is the current LED index being lit (0..99).
  // The spec gives 5 color steps; map 100 LEDs to those 5 buckets.
  int bucket = (level * 5) / NUM_LEDS;   // 0..4
  switch (bucket) {
    case 0: return 0xFF0000;  // red
    case 1: return 0xFFFF00;  // yellow
    case 2: return 0xFFFF55;
    case 3: return 0xFFFFAB;
    default: return 0xFFFFFF; // white
  }
}

// Build a 100-LED frame from an 8-bit PWM duty sample using a log scale.
//   - duty 0 / 255 both represent full deflection from the 128 midpoint
//   - magnitude = |duty - 128| / 128, in (0, 1]
//   - log10(magnitude) ranges roughly from -5.5 (tiny) to 0 (max)
//   - that range is split across 100 LEDs, 1.1 units each.
static void build_frame(uint8_t duty, uint32_t *frame) {
  int deflection = (int)duty - 128;
  if (deflection < 0) deflection = -deflection;   // 0..128

  int lit;
  if (deflection <= 0) {
    lit = 0;
  } else {
    // normalize to (0,1]: treat 1 as the smallest detectable step.
    // log10(deflection/128) is in [-2.1 .. 0] for 8-bit data; the lab
    // spec uses a -5.5..0 range, so we still linearly map onto it.
    double mag = (double)deflection / 128.0;
    double lg  = log10(mag);              // -inf..0
    if (lg < -5.5) lg = -5.5;
    // -5.5 -> 0 LEDs, 0 -> 100 LEDs
    lit = (int)(((lg + 5.5) / 5.5) * (double)NUM_LEDS + 0.5);
    if (lit < 0) lit = 0;
    if (lit > NUM_LEDS) lit = NUM_LEDS;
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < lit) {
      frame[i] = color_for_level(i);
    } else {
      frame[i] = 0x000000;
    }
  }
}

// =====================================================================
// Arduino entry points
// =====================================================================
static uint32_t led_frame[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  // USB-CDC on ESP32-S3 re-enumerates after every reset.  Block until
  // the host (Serial Monitor) actually opens the port, otherwise the
  // boot banner gets printed into the void before the host connects.
  uint32_t wait_start = millis();
  while (!Serial && (millis() - wait_start) < 5000) {
    delay(10);
  }
  delay(500);
  Serial.printf("Lab 4 starting. Sample rate = %u Hz, %u samples\n",
                (unsigned)sampleRate, (unsigned)sizeof(sampleArray));

  Serial.println("TRACE: calling setup_LEDC()");
  Serial.flush();
  setup_LEDC();
  Serial.println("TRACE: setup_LEDC() returned");
  Serial.flush();

  Serial.println("TRACE: calling setup_RMT()");
  Serial.flush();
  setup_RMT();
  Serial.println("TRACE: setup_RMT() returned");
  Serial.flush();

  Serial.println("TRACE: clearing LED frame");
  Serial.flush();
  for (int i = 0; i < NUM_LEDS; i++) led_frame[i] = 0;

  Serial.println("TRACE: calling transmit_led_signal()");
  Serial.flush();
  transmit_led_signal(led_frame);
  Serial.println("TRACE: transmit_led_signal() returned");
  Serial.flush();

  Serial.println("TRACE: setup() complete, entering loop()");
  Serial.flush();
}

void loop() {
  const uint32_t N = sizeof(sampleArray);   // 1 byte per sample

  // Refresh the LED frame every ~50 ms so the visualization is visible
  // without dominating the CPU (which must service the PWM every
  // 1/sampleRate seconds).
  uint32_t samples_per_frame = sampleRate / 20;   // 20 fps
  if (samples_per_frame == 0) samples_per_frame = 1;
  uint32_t frame_counter = 0;

  // Push first sample unconditionally.
  update_PWM(1, sampleArray[0]);

  uint32_t last_print = 0;
  uint32_t spin_count = 0;
  for (uint32_t i = 1; i < N; ) {
    spin_count++;
    if (millis() - last_print > 1000) {
      uint32_t raw = REG_READ(LEDC_INT_RAW_REG);
      Serial.printf("i=%u spins=%u LEDC_INT_RAW=0x%08x\n",
                    (unsigned)i, (unsigned)spin_count, (unsigned)raw);
      last_print = millis();
      spin_count = 0;
    }
    // Spin calling update_PWM; it only actually writes when a PWM
    // period has just completed.  When it does, advance i.
    if (REG_READ(LEDC_INT_RAW_REG) & LEDC_TIMER0_OVF_INT_BIT) {
      update_PWM(0, sampleArray[i]);
      i++;
      if (++frame_counter >= samples_per_frame) {
        frame_counter = 0;
        build_frame(sampleArray[i < N ? i : N - 1], led_frame);
        transmit_led_signal(led_frame);
      }
    }
  }

  // End of song: clear LEDs and pause briefly before looping.
  for (int i = 0; i < NUM_LEDS; i++) led_frame[i] = 0;
  transmit_led_signal(led_frame);
  delay(1000);
}
