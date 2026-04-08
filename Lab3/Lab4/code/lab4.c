#include "lab4.h"

#include <Arduino.h>
#include "esp_rom_sys.h"
#include "array.h"

static inline uint32_t rmt_word(uint16_t d0, uint8_t l0, uint16_t d1, uint8_t l1) {
    return ((uint32_t)l1 << 31) |
           ((uint32_t)d1 << 16) |
           ((uint32_t)l0 << 15) |
           (uint32_t)d0;
}

static inline uint32_t encode_ws2812_bit(uint32_t bit) {
    if (bit) {
        return rmt_word(LAB4_WS_T1H, 1, LAB4_WS_T1L, 0);
    }
    return rmt_word(LAB4_WS_T0H, 1, LAB4_WS_T0L, 0);
}

static void encode_color_grb(uint32_t rgb, uint32_t *out24) {
    uint32_t r = (rgb >> 16) & 0xFFU;
    uint32_t g = (rgb >> 8) & 0xFFU;
    uint32_t b = rgb & 0xFFU;
    uint32_t grb = (g << 16) | (r << 8) | b;

    for (uint32_t bit = 0; bit < LAB4_BITS_PER_LED; bit++) {
        uint32_t value = (grb >> (23U - bit)) & 0x1U;
        out24[bit] = encode_ws2812_bit(value);
    }
}

void setup_RMT(void) {
    REG_SET_BIT(LAB4_DPORT_PERIP_CLK_EN_REG, LAB4_DPORT_RMT_CLK_EN_BIT);
    REG_SET_BIT(LAB4_DPORT_PERIP_RST_EN_REG, LAB4_DPORT_RMT_RST_BIT);
    REG_CLR_BIT(LAB4_DPORT_PERIP_RST_EN_REG, LAB4_DPORT_RMT_RST_BIT);

    uint32_t iomux33 = REG_READ(LAB4_IO_MUX_GPIO33_REG);
    REG_WRITE(LAB4_IO_MUX_GPIO33_REG, (iomux33 & ~LAB4_GPIO_MCU_SEL_MASK) | LAB4_GPIO_MCU_SEL_2);

    REG_WRITE(LAB4_GPIO_FUNC33_OUT_SEL_CFG_REG, LAB4_GPIO_MATRIX_RMT_FUNC);

    uint32_t conf0 = 0;
    conf0 &= ~LAB4_RMT_DIV_CNT_MASK;
    conf0 |= (1U << LAB4_RMT_DIV_CNT_S); // 80 MHz tick; matches WS2812 constants in lab4.h
    conf0 &= ~LAB4_RMT_MEM_SIZE_CH0_MASK;
    conf0 |= (LAB4_RMT_MEM_BLOCKS_CH0 << LAB4_RMT_MEM_SIZE_CH0_S); // use full memory for larger chunk TX
    conf0 &= ~LAB4_RMT_CARRIER_EN_CH0_BIT;
    REG_WRITE(LAB4_RMT_CH0CONF0_REG, conf0);

    uint32_t conf1 = REG_READ(LAB4_RMT_CH0CONF1_REG);
    conf1 |= LAB4_RMT_MEM_OWNER_CH0_BIT;
    conf1 &= ~LAB4_RMT_TX_CONTI_MODE_CH0_BIT;
    conf1 &= ~LAB4_RMT_TX_START_CH0_BIT;
    REG_WRITE(LAB4_RMT_CH0CONF1_REG, conf1);

    REG_SET_BIT(LAB4_RMT_APB_CONF_REG, LAB4_RMT_CONF_UPDATE_CH0_BIT);
}

void setup_LEDC(void) {
    REG_SET_BIT(LAB4_DPORT_PERIP_CLK_EN_REG, LAB4_DPORT_LEDC_CLK_EN_BIT);
    REG_SET_BIT(LAB4_DPORT_PERIP_RST_EN_REG, LAB4_DPORT_LEDC_RST_BIT);
    REG_CLR_BIT(LAB4_DPORT_PERIP_RST_EN_REG, LAB4_DPORT_LEDC_RST_BIT);

    uint32_t iomux14 = REG_READ(LAB4_IO_MUX_GPIO14_REG);
    REG_WRITE(LAB4_IO_MUX_GPIO14_REG, (iomux14 & ~LAB4_GPIO_MCU_SEL_MASK) | LAB4_GPIO_MCU_SEL_2);

    REG_WRITE(LAB4_GPIO_FUNC14_OUT_SEL_CFG_REG, LAB4_GPIO_MATRIX_LEDC_FUNC);

    // Use 8-bit PWM resolution; derive divider so PWM period matches sample rate.
    const uint32_t duty_res_bits = 8U;
    uint32_t divider = (80000000UL << 8) / (sampleRate * (1UL << duty_res_bits));
    if (divider < 256U) {
        divider = 256U;
    }
    if (divider > ((LAB4_LEDC_TIMER_DIV_NUM_MASK) >> LAB4_LEDC_TIMER_DIV_NUM_S)) {
        divider = (LAB4_LEDC_TIMER_DIV_NUM_MASK) >> LAB4_LEDC_TIMER_DIV_NUM_S;
    }

    uint32_t timer_conf = 0;
    timer_conf &= ~LAB4_LEDC_TIMER_DIV_NUM_MASK;
    timer_conf |= (divider << LAB4_LEDC_TIMER_DIV_NUM_S) & LAB4_LEDC_TIMER_DIV_NUM_MASK;
    timer_conf &= ~LAB4_LEDC_TIMER_DUTY_RES_MASK;
    timer_conf |= (duty_res_bits << LAB4_LEDC_TIMER_DUTY_RES_S) & LAB4_LEDC_TIMER_DUTY_RES_MASK;
    timer_conf &= ~LAB4_LEDC_TIMER_PAUSE_BIT;
    timer_conf |= LAB4_LEDC_TIMER_RST_BIT;
    timer_conf |= LAB4_LEDC_TIMER_PARA_UP_BIT;
    REG_WRITE(LAB4_LEDC_HSTIMER0_CONF_REG, timer_conf);
    REG_CLR_BIT(LAB4_LEDC_HSTIMER0_CONF_REG, LAB4_LEDC_TIMER_RST_BIT);

    uint32_t ch_conf0 = REG_READ(LAB4_LEDC_HSCH0_CONF0_REG);
    ch_conf0 &= ~LAB4_LEDC_TIMER_SEL_CH0_MASK;
    ch_conf0 |= (0U << LAB4_LEDC_TIMER_SEL_CH0_S);
    ch_conf0 |= LAB4_LEDC_SIG_OUT_EN_CH0_BIT;
    ch_conf0 |= LAB4_LEDC_PARA_UP_CH0_BIT;
    REG_WRITE(LAB4_LEDC_HSCH0_CONF0_REG, ch_conf0);

    REG_WRITE(LAB4_LEDC_HSCH0_HPOINT_REG, 0U);
    REG_WRITE(LAB4_LEDC_HSCH0_DUTY_REG, 0x80U << 4);

    uint32_t ch_conf1 = REG_READ(LAB4_LEDC_HSCH0_CONF1_REG);
    ch_conf1 |= LAB4_LEDC_DUTY_START_CH0_BIT;
    REG_WRITE(LAB4_LEDC_HSCH0_CONF1_REG, ch_conf1);
}

void update_PWM(uint32_t initial, uint32_t sample) {
    uint32_t ready = REG_READ(LAB4_LEDC_INT_RAW_REG) & LAB4_LEDC_TIMER0_OVF_INT_RAW_BIT;

    if (!initial && !ready) {
        return;
    }

    if (ready) {
        REG_SET_BIT(LAB4_LEDC_INT_CLR_REG, LAB4_LEDC_TIMER0_OVF_INT_CLR_BIT);
    }

    REG_WRITE(LAB4_LEDC_HSCH0_DUTY_REG, (sample & 0xFFU) << 4);
    REG_SET_BIT(LAB4_LEDC_HSCH0_CONF0_REG, LAB4_LEDC_PARA_UP_CH0_BIT);
    REG_SET_BIT(LAB4_LEDC_HSCH0_CONF1_REG, LAB4_LEDC_DUTY_START_CH0_BIT);
}

void transmit_led_signal(uint32_t *colors) {
    uint32_t encoded[LAB4_RMT_TOTAL_WORDS];
    uint32_t led = 0;

    while (led < LAB4_NUM_LEDS) {
        uint32_t chunk_leds = LAB4_NUM_LEDS - led;
        if (chunk_leds > LAB4_RMT_MAX_LEDS_PER_TX) {
            chunk_leds = LAB4_RMT_MAX_LEDS_PER_TX;
        }

        uint32_t out_index = 0;
        for (uint32_t n = 0; n < chunk_leds; n++) {
            encode_color_grb(colors[led + n], &encoded[out_index]);
            out_index += LAB4_BITS_PER_LED;
        }
        encoded[out_index++] = 0U; // terminator

        for (uint32_t i = 0; i < out_index; i++) {
            REG_WRITE(LAB4_RMT_DATA_REG(i), encoded[i]);
        }

        REG_SET_BIT(LAB4_RMT_APB_CONF_REG, LAB4_RMT_CONF_UPDATE_CH0_BIT);
        REG_SET_BIT(LAB4_RMT_CH0CONF1_REG, LAB4_RMT_TX_START_CH0_BIT);

        while (!(REG_READ(LAB4_RMT_INT_RAW_REG) & LAB4_RMT_CH0_TX_END_INT_RAW_BIT)) {
            ;
        }

        REG_SET_BIT(LAB4_RMT_INT_CLR_REG, LAB4_RMT_CH0_TX_END_INT_CLR_BIT);
        led += chunk_leds;
    }

    // WS2812 reset low time.
    esp_rom_delay_us(LAB4_WS_RESET_US);
}