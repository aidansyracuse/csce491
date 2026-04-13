// Lab 4: PWM Audio + Addressable LEDs
// ESP32-S3 register-level drivers.
//
// PIN ASSIGNMENT:
//   LEDC on pin 33  -> LM386 audio amplifier
//   RMT  on pin 14  -> WS2812 chain of 100 LEDs
//
// IDE: Tools -> "USB CDC On Boot" must be ENABLED for Serial.

#include "array.h"

void setup_RMT();
void setup_LEDC();
void update_PWM(uint32_t initial, uint32_t sample);
void transmit_led_signal(uint32_t *colors);

// ============================================================
// Register addresses (ESP32-S3 TRM v1.8, section 37.5)
// ============================================================

#define SYSTEM_PERIP_CLK_EN0_REG   0x600C0018
#define SYSTEM_PERIP_RST_EN0_REG   0x600C0020

#define RMT_BASE                   0x60016000
#define RMT_CH0_CONF0_REG          (RMT_BASE + 0x0020)
#define RMT_CH0_STATUS_REG         (RMT_BASE + 0x0050)   // TRM-confirmed
#define RMT_INT_RAW_REG            (RMT_BASE + 0x0070)
#define RMT_INT_CLR_REG            (RMT_BASE + 0x007C)
#define RMT_SYS_CONF_REG           (RMT_BASE + 0x00C0)
#define RMT_DATE_REG               (RMT_BASE + 0x00CC)   // version reg

// Per TRM 37.3.2.3: "write-starting address of TX channel n is: RMT base + 0x800 + n*48"
#define RMT_CH0_RAM_BASE           (RMT_BASE + 0x0800)

#define LEDC_BASE                  0x60019000
#define LEDC_CH0_CONF0_REG         (LEDC_BASE + 0x0000)
#define LEDC_CH0_DUTY_REG          (LEDC_BASE + 0x0008)
#define LEDC_CH0_CONF1_REG         (LEDC_BASE + 0x000C)
#define LEDC_TIMER0_CONF_REG       (LEDC_BASE + 0x00A0)
#define LEDC_INT_RAW_REG           (LEDC_BASE + 0x00C0)
#define LEDC_INT_CLR_REG           (LEDC_BASE + 0x00CC)
#define LEDC_CONF_REG              (LEDC_BASE + 0x00D0)

#define GPIO_BASE                       0x60004000
#define IO_MUX_BASE                     0x60009000
#define GPIO_ENABLE_REG                 (GPIO_BASE + 0x0020)
#define GPIO_ENABLE1_REG                (GPIO_BASE + 0x002C)
#define GPIO_FUNC_OUT_SEL_CFG_REG(pin)  (GPIO_BASE + 0x0554 + ((pin) * 4))
#define IO_MUX_GPIO_REG(pin)            (IO_MUX_BASE + 0x0010 + ((pin) * 4))

#define LEDC_FUNC_SEL   73
#define RMT_FUNC_SEL    81

#define PIN_LEDC   33
#define PIN_RMT    14

static inline void gpio_enable_output(int pin) {
    if (pin < 32) REG_SET_BIT(GPIO_ENABLE_REG,  (1u << pin));
    else          REG_SET_BIT(GPIO_ENABLE1_REG, (1u << (pin - 32)));
}

// ============================================================
// RMT_SYS_CONF_REG bit layout (verified from TRM register diagram):
//   bit 0   APB_FIFO_MASK       (1 = NONFIFO / direct memory access)
//   bit 1   MEM_CLK_FORCE_ON
//   bit 2   MEM_FORCE_PD
//   bit 3   MEM_FORCE_PU
//   [11:4]  SCLK_DIV_NUM        (0 = divide by 1)
//   [17:12] SCLK_DIV_A          (fractional numerator)
//   [23:18] SCLK_DIV_B          (fractional denominator)
//   [25:24] SCLK_SEL            (1 = APB_CLK)
//   bit 26  SCLK_ACTIVE         <<< rmt_sclk switch (reset = 1, MUST stay on!)
//   bit 31  CLK_EN              (register clock gate)
// ============================================================

#define SYSTEM_RMT_CLK_EN_BIT      (1u << 9)
#define SYSTEM_RMT_RST_BIT         (1u << 9)
#define SYSTEM_LEDC_CLK_EN_BIT     (1u << 11)
#define SYSTEM_LEDC_RST_BIT        (1u << 11)

#define RMT_APB_FIFO_MASK_BIT      (1u << 0)
#define RMT_MEM_CLK_FORCE_ON_BIT   (1u << 1)
#define RMT_MEM_FORCE_PU_BIT       (1u << 3)
#define RMT_SCLK_SEL_SHIFT         24
#define RMT_SCLK_ACTIVE_BIT        (1u << 26)   // <-- the bit I missed
#define RMT_CLK_EN_BIT             (1u << 31)

#define RMT_TX_START_CH0_BIT       (1u << 0)
#define RMT_MEM_RD_RST_CH0_BIT     (1u << 1)
#define RMT_APB_MEM_RST_CH0_BIT    (1u << 2)
#define RMT_IDLE_OUT_LV_CH0_BIT    (1u << 5)
#define RMT_IDLE_OUT_EN_CH0_BIT    (1u << 6)
#define RMT_DIV_CNT_CH0_SHIFT      8
#define RMT_MEM_SIZE_CH0_SHIFT     16
#define RMT_CONF_UPDATE_CH0_BIT    (1u << 24)

#define RMT_CH0_TX_END_INT_BIT     (1u << 0)

#define LEDC_APB_CLK_SEL_BIT       (1u << 0)
#define LEDC_CLK_EN_BIT            (1u << 31)

#define LEDC_TIMER0_DUTY_RES_SHIFT 0
#define LEDC_CLK_DIV_TIMER0_SHIFT  4
#define LEDC_TIMER0_PARA_UP_BIT    (1u << 25)

#define LEDC_SIG_OUT_EN_CH0_BIT    (1u << 2)
#define LEDC_PARA_UP_CH0_BIT       (1u << 4)
#define LEDC_DUTY_START_CH0_BIT    (1u << 31)
#define LEDC_TIMER0_OVF_INT_BIT    (1u << 0)

#define IO_MUX_MCU_SEL_SHIFT       12
#define IO_MUX_FUN_DRV_SHIFT       10

// ============================================================
// WS2812 timing @ 80 MHz
// ============================================================

#define T0H_TICKS  32
#define T0L_TICKS  68
#define T1H_TICKS  64
#define T1L_TICKS  36

static const uint32_t RMT_BIT_0 = (1u << 31) | (T0H_TICKS << 16) | (0u << 15) | T0L_TICKS;
static const uint32_t RMT_BIT_1 = (1u << 31) | (T1H_TICKS << 16) | (0u << 15) | T1L_TICKS;

static uint32_t g_rmt_ch0_conf0_base = 0;

// ============================================================
// LED state
// ============================================================

#define NUM_LEDS        100
#define FRAME_INTERVAL  533

static uint32_t led_colors[NUM_LEDS];
static uint32_t frame_peak    = 0;
static uint32_t frame_counter = 0;

static const uint8_t colormap[5][3] = {
    {255,   0,   0},
    {255, 255,   0},
    {255, 255,  85},
    {255, 255, 171},
    {255, 255, 255}
};

// ============================================================
// RMT setup
// ============================================================

void setup_RMT() {
    REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_RMT_CLK_EN_BIT);
    REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_RMT_RST_BIT);
    REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_RMT_RST_BIT);

    uint32_t sys_conf = 0;
    sys_conf |= RMT_APB_FIFO_MASK_BIT;          // bit 0: direct memory access
    sys_conf |= RMT_MEM_CLK_FORCE_ON_BIT;       // bit 1
    sys_conf |= RMT_MEM_FORCE_PU_BIT;           // bit 3
    sys_conf |= (1u << RMT_SCLK_SEL_SHIFT);     // bits [25:24] = APB_CLK
    sys_conf |= RMT_SCLK_ACTIVE_BIT;            // bit 26: ENABLE rmt_sclk
    sys_conf |= RMT_CLK_EN_BIT;                 // bit 31
    REG_WRITE(RMT_SYS_CONF_REG, sys_conf);

    uint32_t ch0 = 0;
    ch0 |= (1u << RMT_DIV_CNT_CH0_SHIFT);
    ch0 |= (1u << RMT_MEM_SIZE_CH0_SHIFT);
    ch0 |= RMT_IDLE_OUT_EN_CH0_BIT;
    g_rmt_ch0_conf0_base = ch0;

    REG_WRITE(RMT_CH0_CONF0_REG, ch0);
    REG_WRITE(RMT_CH0_CONF0_REG, ch0 | RMT_CONF_UPDATE_CH0_BIT);

    REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_BIT);

    REG_WRITE(IO_MUX_GPIO_REG(PIN_RMT),
              (2u << IO_MUX_MCU_SEL_SHIFT) | (3u << IO_MUX_FUN_DRV_SHIFT));
    REG_WRITE(GPIO_FUNC_OUT_SEL_CFG_REG(PIN_RMT), RMT_FUNC_SEL);
    gpio_enable_output(PIN_RMT);
}

// ============================================================
// LEDC setup
// ============================================================

void setup_LEDC() {
    REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_LEDC_CLK_EN_BIT);
    REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST_BIT);
    REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST_BIT);

    REG_WRITE(LEDC_CONF_REG, LEDC_CLK_EN_BIT | LEDC_APB_CLK_SEL_BIT);

    uint32_t R = sampleRate;
    double div_d = 80000000.0 / (256.0 * (double)R);
    uint32_t div_whole = (uint32_t)div_d;
    uint32_t div_frac  = (uint32_t)((div_d - (double)div_whole) * 256.0 + 0.5);
    if (div_frac > 255) { div_frac = 0; div_whole += 1; }
    uint32_t div_fixed = (div_whole << 8) | div_frac;

    uint32_t timer_conf = 0;
    timer_conf |= (8u << LEDC_TIMER0_DUTY_RES_SHIFT);
    timer_conf |= (div_fixed << LEDC_CLK_DIV_TIMER0_SHIFT);
    timer_conf |= LEDC_TIMER0_PARA_UP_BIT;
    REG_WRITE(LEDC_TIMER0_CONF_REG, timer_conf);

    uint32_t ch_conf0 = LEDC_SIG_OUT_EN_CH0_BIT | LEDC_PARA_UP_CH0_BIT;
    REG_WRITE(LEDC_CH0_CONF0_REG, ch_conf0);
    REG_WRITE(LEDC_CH0_DUTY_REG, (128u << 4));
    REG_WRITE(LEDC_CH0_CONF0_REG, ch_conf0);
    REG_WRITE(LEDC_CH0_CONF1_REG, LEDC_DUTY_START_CH0_BIT);
    REG_WRITE(LEDC_INT_CLR_REG, LEDC_TIMER0_OVF_INT_BIT);

    REG_WRITE(IO_MUX_GPIO_REG(PIN_LEDC),
              (2u << IO_MUX_MCU_SEL_SHIFT) | (3u << IO_MUX_FUN_DRV_SHIFT));
    REG_WRITE(GPIO_FUNC_OUT_SEL_CFG_REG(PIN_LEDC), LEDC_FUNC_SEL);
    gpio_enable_output(PIN_LEDC);
}

// ============================================================
// update_PWM
// ============================================================

void update_PWM(uint32_t initial, uint32_t sample) {
    if (!initial) {
        if (!(REG_READ(LEDC_INT_RAW_REG) & LEDC_TIMER0_OVF_INT_BIT)) return;
        REG_WRITE(LEDC_INT_CLR_REG, LEDC_TIMER0_OVF_INT_BIT);
    }
    REG_WRITE(LEDC_CH0_DUTY_REG, (sample & 0xFF) << 4);
    REG_SET_BIT(LEDC_CH0_CONF0_REG, LEDC_PARA_UP_CH0_BIT);
    REG_SET_BIT(LEDC_CH0_CONF1_REG, LEDC_DUTY_START_CH0_BIT);
}

// ============================================================
// Send one 24-bit GRB color
// ============================================================

static int rmt_send_color(uint32_t grb) {
    volatile uint32_t *ram = (volatile uint32_t *)RMT_CH0_RAM_BASE;

    for (int i = 0; i < 24; i++) {
        uint32_t bit = (grb >> (23 - i)) & 1u;
        ram[i] = bit ? RMT_BIT_1 : RMT_BIT_0;
    }
    ram[24] = 0;

    REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_MEM_RD_RST_CH0_BIT);
    REG_CLR_BIT(RMT_CH0_CONF0_REG, RMT_MEM_RD_RST_CH0_BIT);
    REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_APB_MEM_RST_CH0_BIT);
    REG_CLR_BIT(RMT_CH0_CONF0_REG, RMT_APB_MEM_RST_CH0_BIT);

    REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_BIT);

    REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_CONF_UPDATE_CH0_BIT);
    REG_SET_BIT(RMT_CH0_CONF0_REG, RMT_TX_START_CH0_BIT);

    uint32_t timeout = 2000;
    while (!(REG_READ(RMT_INT_RAW_REG) & RMT_CH0_TX_END_INT_BIT)) {
        if (--timeout == 0) return 0;
    }
    REG_WRITE(RMT_INT_CLR_REG, RMT_CH0_TX_END_INT_BIT);
    return 1;
}

// ============================================================
// transmit_led_signal
// ============================================================

void transmit_led_signal(uint32_t *colors) {
    for (int i = 0; i < NUM_LEDS; i++) {
        uint32_t c = colors[i];
        uint8_t r = (c >> 16) & 0xFF;
        uint8_t g = (c >>  8) & 0xFF;
        uint8_t b =  c        & 0xFF;
        uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
        rmt_send_color(grb);
    }
    delayMicroseconds(80);
}

// ============================================================
// VU-bar builder
// ============================================================

static void build_colors(uint32_t amplitude) {
    uint32_t lit = (amplitude * NUM_LEDS) / 128;
    if (lit > NUM_LEDS) lit = NUM_LEDS;

    for (int i = 0; i < NUM_LEDS; i++) {
        if ((uint32_t)i < lit) {
            int bucket = (lit > 0) ? (int)((i * 5) / lit) : 0;
            if (bucket > 4) bucket = 4;
            uint8_t r = colormap[bucket][0] >> 3;   // /8 brightness
            uint8_t g = colormap[bucket][1] >> 3;
            uint8_t b = colormap[bucket][2] >> 3;
            led_colors[i] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        } else {
            led_colors[i] = 0;
        }
    }
}

// ============================================================
// LED self-test
// ============================================================

static void led_self_test() {
    for (int i = 0; i < NUM_LEDS; i++) led_colors[i] = 0x00200000;
    transmit_led_signal(led_colors); delay(400);
    for (int i = 0; i < NUM_LEDS; i++) led_colors[i] = 0x00002000;
    transmit_led_signal(led_colors); delay(400);
    for (int i = 0; i < NUM_LEDS; i++) led_colors[i] = 0x00000020;
    transmit_led_signal(led_colors); delay(400);
    for (int pos = 0; pos < NUM_LEDS; pos++) {
        for (int i = 0; i < NUM_LEDS; i++) led_colors[i] = 0;
        led_colors[pos] = 0x00101010;
        transmit_led_signal(led_colors);
        delay(8);
    }
    for (int i = 0; i < NUM_LEDS; i++) led_colors[i] = 0;
    transmit_led_signal(led_colors);
}

// ============================================================
// Arduino entry
// ============================================================

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 5000) delay(10);
    delay(500);

    Serial.println();
    Serial.println("=== Lab 4 boot ===");
    Serial.printf("sampleRate=%lu, N samples=%lu\n",
        (unsigned long)sampleRate,
        (unsigned long)(sizeof(sampleArray) / sizeof(sampleArray[0])));

    setup_LEDC();
    setup_RMT();

    Serial.printf("RMT_SYS_CONF     = 0x%08lX  (expect bit 26 SCLK_ACTIVE set now)\n",
                  (unsigned long)REG_READ(RMT_SYS_CONF_REG));
    Serial.printf("RMT_CH0_CONF0    = 0x%08lX\n", (unsigned long)REG_READ(RMT_CH0_CONF0_REG));
    Serial.printf("RMT_CH0_STATUS   = 0x%08lX\n", (unsigned long)REG_READ(RMT_CH0_STATUS_REG));
    Serial.printf("RMT_DATE         = 0x%08lX  (version reg, should be nonzero)\n",
                  (unsigned long)REG_READ(RMT_DATE_REG));
    Serial.flush();

    uint32_t t_start = micros();
    int ok = rmt_send_color(0x002000);
    uint32_t t_elapsed = micros() - t_start;
    Serial.printf("Single rmt_send_color: %s in %lu us (STATUS=0x%08lX, INT_RAW=0x%08lX)\n",
                  ok ? "OK" : "TIMEOUT",
                  (unsigned long)t_elapsed,
                  (unsigned long)REG_READ(RMT_CH0_STATUS_REG),
                  (unsigned long)REG_READ(RMT_INT_RAW_REG));
    Serial.flush();

    Serial.println("Running LED self-test...");
    Serial.flush();
    led_self_test();
    Serial.println("Self-test done");

    update_PWM(1, sampleArray[0]);
    Serial.println("PWM primed. Entering audio loop.");
    Serial.flush();
}

void loop() {
    const uint32_t N = sizeof(sampleArray) / sizeof(sampleArray[0]);
    uint32_t last_print = millis();
    uint32_t advances  = 0;

    for (uint32_t i = 1; i < N; ) {
        if (millis() - last_print > 1000) {
            Serial.printf("i=%lu advanced=%lu/s\n",
                (unsigned long)i, (unsigned long)advances);
            last_print = millis();
            advances = 0;
        }
        if (REG_READ(LEDC_INT_RAW_REG) & LEDC_TIMER0_OVF_INT_BIT) {
            update_PWM(0, sampleArray[i]);
            int32_t s = (int32_t)sampleArray[i] - 128;
            uint32_t mag = (s < 0) ? (uint32_t)(-s) : (uint32_t)s;
            if (mag > frame_peak) frame_peak = mag;
            if (++frame_counter >= FRAME_INTERVAL) {
                frame_counter = 0;
                build_colors(frame_peak);
                transmit_led_signal(led_colors);
                frame_peak = 0;
            }
            i++; advances++;
        }
    }

    Serial.println("=== audio done ===");
    for (int i = 0; i < NUM_LEDS; i++) led_colors[i] = 0;
    transmit_led_signal(led_colors);
    while (1) delay(1000);
}