#include "array.h"
#include "lab4.h"
#include <Arduino.h>

static uint32_t ledColors[LAB4_NUM_LEDS];
static uint32_t sampleIndex = 0;
static uint32_t visualDivider = 0;

static uint32_t magnitude_to_led_count(uint8_t sample) {
    int32_t centered = (int32_t)sample - 128;
    if (centered < 0) centered = -centered;

    if (centered > 96) return 100;
    if (centered > 80) return 85;
    if (centered > 64) return 70;
    if (centered > 48) return 55;
    if (centered > 32) return 40;
    if (centered > 16) return 25;
    if (centered > 8)  return 12;
    return 4;
}

static uint32_t heat_color(uint32_t idx, uint32_t lit) {
    if (idx >= lit) return 0x000000;
    uint32_t t = (idx * 255U) / (lit == 0 ? 1 : lit);
    uint32_t r = t;
    uint32_t g = 255U - (t / 2U);
    uint32_t b = 0U;
    return (r << 16) | (g << 8) | b;
}

static void update_visualizer(uint8_t sample) {
    uint32_t lit = magnitude_to_led_count(sample);
    for (uint32_t i = 0; i < LAB4_NUM_LEDS; i++) {
        ledColors[i] = heat_color(i, lit);
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("=== Lab 4 starting ===");
    Serial.flush();

    Serial.println("Calling setup_RMT()...");
    Serial.flush();
    setup_RMT();
    Serial.println("setup_RMT() done");
    Serial.flush();

    Serial.println("Calling setup_LEDC()...");
    Serial.flush();
    setup_LEDC();
    Serial.println("setup_LEDC() done");
    Serial.flush();

    Serial.println("Clearing LEDs...");
    Serial.flush();
    for (uint32_t i = 0; i < LAB4_NUM_LEDS; i++) ledColors[i] = 0x000000;
    transmit_led_signal(ledColors);
    Serial.println("LED clear done");
    Serial.flush();

    // Flash all LEDs red briefly so we can confirm RMT is working
    Serial.println("Flashing LEDs red for 2 seconds...");
    Serial.flush();
    for (uint32_t i = 0; i < LAB4_NUM_LEDS; i++) ledColors[i] = 0xFF0000;
    transmit_led_signal(ledColors);
    delay(2000);
    for (uint32_t i = 0; i < LAB4_NUM_LEDS; i++) ledColors[i] = 0x000000;
    transmit_led_signal(ledColors);
    Serial.println("LED flash done");
    Serial.flush();

    Serial.printf("Starting PWM. sampleRate=%u, samples=%u\n",
                  (unsigned)sampleRate, (unsigned)sizeof(sampleArray));
    Serial.flush();
    update_PWM(1, sampleArray[0]);

    Serial.println("=== setup() complete ===");
    Serial.flush();
}

void loop() {
    uint8_t sample = sampleArray[sampleIndex];
    update_PWM(0, sample);

    visualDivider++;
    if (visualDivider >= 250U) {
        visualDivider = 0;
        update_visualizer(sample);
        transmit_led_signal(ledColors);
    }

    sampleIndex++;
    if (sampleIndex >= (sizeof(sampleArray) / sizeof(sampleArray[0]))) {
        sampleIndex = 0;
    }
}
