#include "lab4.h"
#include <Arduino.h>
#include "esp_rom_sys.h"
#include "array.h"

// =====================================================================
// RMT word packing
// Bit 31    = level1
// Bits 30:16 = period1
// Bit 15    = level0
// Bits 14:0  = period0
// =====================================================================
static inline uint32_t rmt_word(uint16_t p0, uint8_t l0, uint16_t p1, uint8_t l1) {
    return ((uint32_t)(l1 & 1U) << 31) |
           ((uint32_t)(p1 & 0x7FFFU) << 16) |
           ((uint32_t)(l0 & 1U) << 15) |
           (uint32_t)(p0 & 0x7FFFU);
}

static inline uint32_t encode_ws2812_bit(uint32_t bit) {
    if (bit) {
        return rmt_word(LAB4_WS_T1H, 1, LAB4_WS_T1L, 0);
    }
    return rmt_word(LAB4_WS_T0H, 1, LAB4_WS_T0L, 0);
}

static void encode_color_grb(uint32_t rgb, uint32_t *out24) {
    uint32_t r   = (rgb >> 16) & 0xFFU;
    uint32_t g   = (rgb >>  8) & 0xFFU;
    uint32_t b   =  rgb        & 0xFFU;
    uint32_t grb = (g << 16) | (r << 8) | b;
    for (uint32_t bit = 0; bit < LAB4_BITS_PER_LED; bit++) {
        out24[bit] = encode_ws2812_bit((grb >> (23U - bit)) & 1U);
    }
}

// =====================================================================
// setup_RMT
// =====================================================================
void setup_RMT(void) {
    // 1. Enable RMT clock, assert then deassert reset
    REG_SET_BIT(LAB4_DPORT_PERIP_CLK_EN_REG, LAB4_DPORT_RMT_CLK_EN_BIT);
    REG_SET_BIT(LAB4_DPORT_PERIP_RST_EN_REG, LAB4_DPORT_RMT_RST_BIT);
    REG_CLR_BIT(LAB4_DPORT_PERIP_RST_EN_REG, LAB4_DPORT_RMT_RST_BIT);

    // 2. RMT_SYS_CONF_REG
    //    - RMT_CLK_EN (bit 31) = 1
    //    - RMT_MEM_CLK_FORCE_ON (bit 1) = 1
    //    - RMT_SCLK_SEL (bits 25:24) = 1  (APB = 80 MHz)
    //    - RMT_SCLK_DIV_NUM (bits 11:4) = 0  (divider = NUM+1 = 1)
    //    - RMT_APB_FIFO_MASK (bit 0) = 0
    uint32_t sys_conf = 0;
    sys_conf |= LAB4_RMT_CLK_EN_BIT;
    sys_conf |= LAB4_RMT_MEM_CLK_FORCE_ON_BIT;
    sys_conf |= (1U << LAB4_RMT_SCLK_SEL_S);
    sys_conf |= (0U << LAB4_RMT_SCLK_DIV_NUM_S);
    REG_WRITE(LAB4_RMT_SYS_CONF_REG, sys_conf);

    // 3. RMT_CH0_CONF0_REG
    //    - RMT_DIV_CNT_CH0 (bits 15:8) = 1
    //    - RMT_MEM_SIZE_CH0 (bits 19:16) = 1  (1 block)
    //    - RMT_CARRIER_EN_CH0 (bit 20) = 0
    //    - RMT_IDLE_OUT_EN_CH0 (bit 6) = 1  (drive idle level)
    //    - RMT_IDLE_OUT_LV_CH0 (bit 5) = 0  (idle low = WS2812 reset)
    //    - RMT_CONF_UPDATE_CH0 (bit 24) = 1
    uint32_t ch0 = 0;
    ch0 |= (1U << LAB4_RMT_DIV_CNT_CH0_S);
    ch0 |= (1U << LAB4_RMT_MEM_SIZE_CH0_S);
    ch0 |= LAB4_RMT_IDLE_OUT_EN_CH0_BIT;
    ch0 |= LAB4_RMT_CONF_UPDATE_CH0_BIT;
    REG_WRITE(LAB4_RMT_CH0CONF0_REG, ch0);

    // 4. Clear stale TX-end interrupt
    REG_WRITE(LAB4_RMT_INT_CLR_REG, LAB4_RMT_CH0_TX_END_INT_CLR_BIT);

    // 5. Connect GPIO33 to RMT ch0 output
    //    IO_MUX: MCU_SEL=2 (GPIO matrix), drive strength=3
    REG_WRITE(LAB4_IO_MUX_GPIO33_REG,
              (2U << LAB4_GPIO_MCU_SEL_SHIFT) | (3U << LAB4_GPIO_FUN_DRV_SHIFT));
    //    GPIO matrix: route RMT signal (81) to pin 33
    REG_WRITE(LAB4_GPIO_FUNC33_OUT_SEL_CFG_REG, LAB4_GPIO_MATRIX_RMT_FUNC);
    //    Enable GPIO33 as output (pin 33 -> bit 1 of GPIO_ENABLE1_REG)
    REG_SET_BIT(LAB4_GPIO_ENABLE1_REG, BIT(33 - 32));
}

// =====================================================================
// setup_LEDC
// =====================================================================
void setup_LEDC(void) {
    // 1. Enable LEDC clock, assert then deassert reset
    REG_SET_BIT(LAB4_DPORT_PERIP_CLK_EN_REG, LAB4_DPORT_LEDC_CLK_EN_BIT);
    REG_SET_BIT(LAB4_DPORT_PERIP_RST_EN_REG, LAB4_DPORT_LEDC_RST_BIT);
    REG_CLR_BIT(LAB4_DPORT_PERIP_RST_EN_REG, LAB4_DPORT_LEDC_RST_BIT);

    // 2. LEDC_CONF_REG: enable register clock (bit 31), select APB clock (bit 0)
    REG_WRITE(LAB4_LEDC_CONF_REG, LAB4_LEDC_CLK_EN_BIT | LAB4_LEDC_APB_CLK_SEL_BIT);

    // 3. LEDC_TIMER0_CONF_REG
    //    Divider is 10.8 fixed-point: whole = floor(80e6/R), frac = (80e6/R - whole)*256
    //    Packed as: divider[17:8]=whole, divider[7:0]=frac -> shift left by LAB4_LEDC_CLK_DIV_TIMER0_S=4
    uint32_t R         = sampleRate;
    uint32_t div_whole = 80000000UL / R;
    uint32_t div_frac  = (uint32_t)(((80000000ULL * 256ULL) / (uint64_t)R) & 0xFFU);
    uint32_t divider   = (div_whole << 8) | div_frac;   // 18-bit 10.8 value

    uint32_t timer_conf = 0;
    timer_conf |= (8U      << LAB4_LEDC_DUTY_RES_S);       // 8-bit duty resolution
    timer_conf |= (divider << LAB4_LEDC_CLK_DIV_TIMER0_S); // clock divider
    // LEDC_TIMER0_RST (bit 23) = 0 (not in reset)
    timer_conf |= LAB4_LEDC_TIMER0_PARA_UP_BIT;            // commit (bit 25)
    REG_WRITE(LAB4_LEDC_TIMER0_CONF_REG, timer_conf);

    // 4. LEDC_CH0_CONF0_REG
    //    - LEDC_TIMER_SEL_CH0 (bits 1:0) = 0  (use timer 0)
    //    - LEDC_SIG_OUT_EN_CH0 (bit 2)   = 1  (enable output)
    //    - LEDC_PARA_UP_CH0 (bit 4)      = 1  (commit)
    uint32_t ch_conf0 = 0;
    ch_conf0 |= (0U << LAB4_LEDC_TIMER_SEL_CH0_S);
    ch_conf0 |= LAB4_LEDC_SIG_OUT_EN_CH0_BIT;
    ch_conf0 |= LAB4_LEDC_PARA_UP_CH0_BIT;
    REG_WRITE(LAB4_LEDC_CH0_CONF0_REG, ch_conf0);

    // 5. Set initial duty to mid-scale (128 shifted left 4 for fraction bits)
    REG_WRITE(LAB4_LEDC_CH0_DUTY_REG, 0x80U << 4);

    // 6. LEDC_CH0_CONF1_REG: set LEDC_DUTY_START_CH0 (bit 31) to start PWM
    REG_SET_BIT(LAB4_LEDC_CH0_CONF1_REG, LAB4_LEDC_DUTY_START_CH0_BIT);

    // 7. Connect GPIO14 to LEDC ch0 output
    REG_WRITE(LAB4_IO_MUX_GPIO14_REG,
              (2U << LAB4_GPIO_MCU_SEL_SHIFT) | (3U << LAB4_GPIO_FUN_DRV_SHIFT));
    REG_WRITE(LAB4_GPIO_FUNC14_OUT_SEL_CFG_REG, LAB4_GPIO_MATRIX_LEDC_FUNC);
}

// =====================================================================
// update_PWM
// =====================================================================
void update_PWM(uint32_t initial, uint32_t sample) {
    uint32_t ready = REG_READ(LAB4_LEDC_INT_RAW_REG) & LAB4_LEDC_TIMER0_OVF_INT_RAW_BIT;

    if (!initial && !ready) {
        return;
    }

    if (ready) {
        REG_SET_BIT(LAB4_LEDC_INT_CLR_REG, LAB4_LEDC_TIMER0_OVF_INT_CLR_BIT);
    }

    // Duty value occupies bits [11:4]; shift sample left by 4
    REG_WRITE(LAB4_LEDC_CH0_DUTY_REG, (sample & 0xFFU) << 4);
    REG_SET_BIT(LAB4_LEDC_CH0_CONF0_REG, LAB4_LEDC_PARA_UP_CH0_BIT);
    REG_SET_BIT(LAB4_LEDC_CH0_CONF1_REG, LAB4_LEDC_DUTY_START_CH0_BIT);
}

// =====================================================================
// transmit_led_signal
//   colors[i] = 0x00RRGGBB
//   Sends one LED at a time: 24 RMT words + zero terminator
// =====================================================================
void transmit_led_signal(uint32_t *colors) {
    uint32_t encoded[LAB4_BITS_PER_LED + 1U];

    for (uint32_t led = 0; led < LAB4_NUM_LEDS; led++) {
        encode_color_grb(colors[led], encoded);
        encoded[LAB4_BITS_PER_LED] = 0U;  // zero terminator

        // Write to RMT RAM
        for (uint32_t i = 0; i <= LAB4_BITS_PER_LED; i++) {
            REG_WRITE(LAB4_RMT_DATA_REG(i), encoded[i]);
        }

        // Reset read pointer, commit config, start TX
        REG_SET_BIT(LAB4_RMT_CH0CONF0_REG, LAB4_RMT_MEM_RD_RST_CH0_BIT);
        REG_CLR_BIT(LAB4_RMT_CH0CONF0_REG, LAB4_RMT_MEM_RD_RST_CH0_BIT);
        REG_SET_BIT(LAB4_RMT_CH0CONF0_REG, LAB4_RMT_CONF_UPDATE_CH0_BIT);
        REG_SET_BIT(LAB4_RMT_CH0CONF0_REG, LAB4_RMT_TX_START_CH0_BIT);

        // Wait for TX complete
        while (!(REG_READ(LAB4_RMT_INT_RAW_REG) & LAB4_RMT_CH0_TX_END_INT_RAW_BIT)) {
            ;
        }
        REG_WRITE(LAB4_RMT_INT_CLR_REG, LAB4_RMT_CH0_TX_END_INT_CLR_BIT);
    }

    // WS2812 latch: hold line low > 50 us
    esp_rom_delay_us(LAB4_WS_RESET_US);
}
