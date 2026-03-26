# Lab 4 Starter Project (RMT + LEDC Audio Visualizer)

This project was created in the current sandbox at:

`c:\Users\randa\Downloads\csce491\Lab3\Lab4`

## Folder Layout

- `code/code.ino` : main loop + audio visualizer mapping
- `code/lab4.h` : register `#define`s + required function prototypes
- `code/lab4.c` : low-level RMT and LEDC driver functions
- `code/array.h` : placeholder audio array/sample rate
- `audioToPCM.py` : wav -> `array.h` conversion script

## Required Functions Included

- `setup_RMT()`
- `setup_LEDC()`
- `update_PWM(uint32_t initial, uint32_t sample)`
- `transmit_led_signal(uint32_t *colors)`

## How To Use

1. Convert a small wav first:
   `python audioToPCM.py mySong.wav`
2. The script writes directly to `code/array.h`.
3. Open `code/code.ino` in Arduino IDE.
4. Select ESP32 board + correct COM port.
5. Upload and test RMT LEDs and PWM audio.

## Notes

- WS2812 timing constants are set for 80 MHz APB timing assumptions.
- GPIO matrix function IDs match your lab notes: LEDC=73 (GPIO14), RMT=81 (GPIO33).
- Visualizer updates at a lower rate than audio updates to keep playback stable.