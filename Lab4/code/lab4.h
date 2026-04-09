#ifndef LAB4_H
#define LAB4_H

#include <stdint.h>
#include "soc/soc.h"

// ===== Clock/Reset control (ESP32-S3) =====
#define LAB4_DPORT_PERIP_CLK_EN_REG       0x600C0018U
#define LAB4_DPORT_PERIP_RST_EN_REG       0x600C0020U
#define LAB4_DPORT_RMT_CLK_EN_BIT         BIT(9)
#define LAB4_DPORT_LEDC_CLK_EN_BIT        BIT(11)
#define LAB4_DPORT_RMT_RST_BIT            BIT(9)
#define LAB4_DPORT_LEDC_RST_BIT           BIT(11)

// ===== GPIO matrix (ESP32-S3) =====
// GPIO_FUNCx_OUT_SEL_CFG_REG = 0x60004000 + 0x0554 + (pin * 4)
#define LAB4_GPIO_FUNC14_OUT_SEL_CFG_REG  (0x60004000U + 0x0554U + (14U * 4U))
#define LAB4_GPIO_FUNC33_OUT_SEL_CFG_REG  (0x60004000U + 0x0554U + (33U * 4U))
#define LAB4_GPIO_ENABLE1_REG             (0x60004000U + 0x002CU)
#define LAB4_GPIO_MATRIX_LEDC_FUNC        73U
#define LAB4_GPIO_MATRIX_RMT_FUNC         81U

// ===== IO MUX (ESP32-S3) =====
// IO_MUX_GPIOn_REG = 0x60009000 + 0x0010 + (pin * 4)
#define LAB4_IO_MUX_GPIO14_REG            (0x60009000U + 0x0010U + (14U * 4U))
#define LAB4_IO_MUX_GPIO33_REG            (0x60009000U + 0x0010U + (33U * 4U))
#define LAB4_GPIO_MCU_SEL_SHIFT           12U
#define LAB4_GPIO_FUN_DRV_SHIFT           10U

// ===== RMT (ESP32-S3) =====
#define LAB4_RMT_CH0CONF0_REG             0x60016020U
#define LAB4_RMT_INT_RAW_REG              0x60016070U
#define LAB4_RMT_INT_CLR_REG              0x6001607CU
#define LAB4_RMT_SYS_CONF_REG             0x600160C0U
#define LAB4_RMT_RAM_BASE                 0x60016800U
#define LAB4_RMT_DATA_REG(i)              (LAB4_RMT_RAM_BASE + ((i) * 4U))

// RMT_SYS_CONF_REG bits
#define LAB4_RMT_CLK_EN_BIT               BIT(31)
#define LAB4_RMT_MEM_CLK_FORCE_ON_BIT     BIT(1)
#define LAB4_RMT_SCLK_DIV_NUM_S           4U
#define LAB4_RMT_SCLK_SEL_S               24U

// RMT_CH0_CONF0_REG bits
#define LAB4_RMT_TX_START_CH0_BIT         BIT(0)
#define LAB4_RMT_MEM_RD_RST_CH0_BIT       BIT(1)
#define LAB4_RMT_DIV_CNT_CH0_S            8U
#define LAB4_RMT_MEM_SIZE_CH0_S           16U
#define LAB4_RMT_IDLE_OUT_LV_CH0_BIT      BIT(5)
#define LAB4_RMT_IDLE_OUT_EN_CH0_BIT      BIT(6)
#define LAB4_RMT_CARRIER_EN_CH0_BIT       BIT(20)
#define LAB4_RMT_CONF_UPDATE_CH0_BIT      BIT(24)

// RMT interrupt bits
#define LAB4_RMT_CH0_TX_END_INT_RAW_BIT   BIT(0)
#define LAB4_RMT_CH0_TX_END_INT_CLR_BIT   BIT(0)

// ===== LEDC (ESP32-S3) =====
#define LAB4_LEDC_CH0_CONF0_REG           0x60019000U
#define LAB4_LEDC_CH0_DUTY_REG            0x60019008U
#define LAB4_LEDC_CH0_CONF1_REG           0x6001900CU
#define LAB4_LEDC_TIMER0_CONF_REG         0x600190A0U
#define LAB4_LEDC_INT_RAW_REG             0x600190C0U
#define LAB4_LEDC_INT_CLR_REG             0x600190CCU
#define LAB4_LEDC_CONF_REG                0x600190D0U

// LEDC_CONF_REG bits
#define LAB4_LEDC_CLK_EN_BIT              BIT(31)
#define LAB4_LEDC_APB_CLK_SEL_BIT         BIT(0)

// LEDC_TIMER0_CONF_REG bits
#define LAB4_LEDC_DUTY_RES_S              0U
#define LAB4_LEDC_CLK_DIV_TIMER0_S        4U
#define LAB4_LEDC_TIMER0_RST_BIT          BIT(23)
#define LAB4_LEDC_TIMER0_PARA_UP_BIT      BIT(25)

// LEDC_CH0_CONF0_REG bits
#define LAB4_LEDC_TIMER_SEL_CH0_S         0U
#define LAB4_LEDC_SIG_OUT_EN_CH0_BIT      BIT(2)
#define LAB4_LEDC_PARA_UP_CH0_BIT         BIT(4)

// LEDC_CH0_CONF1_REG bits
#define LAB4_LEDC_DUTY_START_CH0_BIT      BIT(31)

// LEDC interrupt bits
#define LAB4_LEDC_TIMER0_OVF_INT_RAW_BIT  BIT(0)
#define LAB4_LEDC_TIMER0_OVF_INT_CLR_BIT  BIT(0)

// ===== WS2812 timing at 80 MHz (12.5 ns/tick) =====
#define LAB4_WS_T0H       32U   // 400 ns
#define LAB4_WS_T0L       68U   // 850 ns
#define LAB4_WS_T1H       64U   // 800 ns
#define LAB4_WS_T1L       36U   // 450 ns
#define LAB4_WS_RESET_US  60U

#define LAB4_NUM_LEDS      100U
#define LAB4_BITS_PER_LED   24U

#ifdef __cplusplus
extern "C" {
#endif
void setup_RMT(void);
void setup_LEDC(void);
void update_PWM(uint32_t initial, uint32_t sample);
void transmit_led_signal(uint32_t *colors);
#ifdef __cplusplus
}
#endif

#endif
