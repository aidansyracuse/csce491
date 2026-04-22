// Lab 5: Motor Control
// ESP32-S3 — LEDC PWM motor drive, PCNT quadrature encoder, PID control
//
// Pin assignment (Motor 1):
//   GPIO 5  -> PWM1    (LEDC output to motor driver)
//   GPIO 4  -> DIR1    (direction, held low)
//   GPIO 21 -> BRAKE   (held low)
//   GPIO 6  -> ENC_A1  (PCNT input, 100 pulses/rev, rising edge)
//
// Architecture:
//   Task A (10 ms): read PCNT, compute speed (pulses/sec)
//   Task B (10 ms): PID controller, update LEDC duty
//   loop() (1 s):   report CPU utilization
//
// IDE: Tools -> "USB CDC On Boot" -> Enabled

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h"

// ============================================================
// Pin definitions
// ============================================================

#define PIN_PWM     5
#define PIN_DIR     4
#define PIN_BRAKE   21
#define PIN_ENC_A   6

// ============================================================
// Register addresses (carried from Lab 4, verified against TRM)
// ============================================================

#define SYSTEM_PERIP_CLK_EN0_REG   0x600C0018
#define SYSTEM_PERIP_RST_EN0_REG   0x600C0020

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
#define GPIO_FUNC_OUT_SEL_CFG_REG(pin)  (GPIO_BASE + 0x0554 + ((pin) * 4))
#define IO_MUX_GPIO_REG(pin)            (IO_MUX_BASE + 0x0010 + ((pin) * 4))

#define LEDC_FUNC_SEL   73

// ============================================================
// LEDC bit masks
// ============================================================

#define SYSTEM_LEDC_CLK_EN_BIT     (1u << 11)
#define SYSTEM_LEDC_RST_BIT        (1u << 11)

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
// Motor / PID parameters
// ============================================================

#define MOTOR_PWM_FREQ       20000   // 20 kHz
#define MOTOR_DUTY_BITS      8       // 0..255
#define PULSES_PER_REV       100

#define TASK_PERIOD_MS       10      // 10 ms for both tasks
#define TASK_PERIOD_SEC      (TASK_PERIOD_MS / 1000.0f)

#define TARGET_RPM           1500.0f // setpoint — easy to change

// PID gains — start conservative, tune on hardware
static float Kp = 0.3f;
static float Ki = 0.05f;
static float Kd = 0.01f;

// ============================================================
// Shared state (written by task A, read by task B)
// ============================================================

static volatile float motor_speed = 0;     // measured speed in pulses/sec
static volatile float motor_rpm   = 0;     // measured RPM

// PID state (internal to task B)
static float integral_error = 0;
static float prev_error     = 0;
static float pwm_output     = 0;           // duty cycle 0..255

// CPU utilization tracking
static volatile uint64_t taskA_total = 0;
static volatile uint64_t taskB_total = 0;
static uint64_t start_time = 0;

// PCNT handles
static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_channel_handle_t pcnt_channel = NULL;

// ============================================================
// LEDC setup (register-level, 20 kHz, 8-bit, on PIN_PWM)
// ============================================================

void setup_LEDC() {
    REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_LEDC_CLK_EN_BIT);
    REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST_BIT);
    REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_LEDC_RST_BIT);

    REG_WRITE(LEDC_CONF_REG, LEDC_CLK_EN_BIT | LEDC_APB_CLK_SEL_BIT);

    // Clock divider for 20 kHz @ 8-bit resolution:
    //   divider = 80e6 / (20000 * 256) = 15.625
    //   10.8 fixed: whole=15, frac=round(0.625*256)=160
    //   packed = (15 << 8) | 160 = 4000
    uint32_t div_fixed = (15u << 8) | 160u;

    uint32_t timer_conf = 0;
    timer_conf |= (MOTOR_DUTY_BITS << LEDC_TIMER0_DUTY_RES_SHIFT);
    timer_conf |= (div_fixed << LEDC_CLK_DIV_TIMER0_SHIFT);
    timer_conf |= LEDC_TIMER0_PARA_UP_BIT;
    REG_WRITE(LEDC_TIMER0_CONF_REG, timer_conf);

    uint32_t ch_conf0 = LEDC_SIG_OUT_EN_CH0_BIT | LEDC_PARA_UP_CH0_BIT;
    REG_WRITE(LEDC_CH0_CONF0_REG, ch_conf0);

    // Start at duty = 0 (motor stopped)
    REG_WRITE(LEDC_CH0_DUTY_REG, 0);
    REG_WRITE(LEDC_CH0_CONF0_REG, ch_conf0);
    REG_WRITE(LEDC_CH0_CONF1_REG, LEDC_DUTY_START_CH0_BIT);
    REG_WRITE(LEDC_INT_CLR_REG, LEDC_TIMER0_OVF_INT_BIT);

    // Route to pin
    REG_WRITE(IO_MUX_GPIO_REG(PIN_PWM),
              (2u << IO_MUX_MCU_SEL_SHIFT) | (3u << IO_MUX_FUN_DRV_SHIFT));
    REG_WRITE(GPIO_FUNC_OUT_SEL_CFG_REG(PIN_PWM), LEDC_FUNC_SEL);
    REG_SET_BIT(GPIO_ENABLE_REG, (1u << PIN_PWM));
}

// ============================================================
// Set LEDC duty (0..255)
// ============================================================

static void set_duty(uint32_t duty) {
    if (duty > 255) duty = 255;
    REG_WRITE(LEDC_CH0_DUTY_REG, duty << 4);
    REG_SET_BIT(LEDC_CH0_CONF0_REG, LEDC_PARA_UP_CH0_BIT);
    REG_SET_BIT(LEDC_CH0_CONF1_REG, LEDC_DUTY_START_CH0_BIT);
}

// ============================================================
// PCNT setup (ESP-IDF driver, ENC_A1 rising edge)
// ============================================================

void setup_PCNT() {
    pcnt_unit_config_t unit_config = {
        .low_limit  = -1,
        .high_limit = (1 << 14),
        .intr_priority = 0,
        .flags = { .accum_count = 0 }
    };

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num  = PIN_ENC_A,
        .level_gpio_num = -1,
        .flags = {
            .invert_edge_input  = 0,
            .invert_level_input = 0,
            .virt_edge_io_level = 0,
            .io_loop_back       = 0
        }
    };

    pcnt_new_unit(&unit_config, &pcnt_unit);
    pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_channel);
    pcnt_channel_set_edge_action(pcnt_channel,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
}

// ============================================================
// Task A: Speed Monitor (10 ms period)
//   Reads PCNT, computes speed in pulses/sec and RPM.
// ============================================================

void speed_monitor_task(void *args) {
    uint64_t t_start;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_MS);

    for (;;) {
        t_start = esp_timer_get_time();

        int count = 0;
        pcnt_unit_get_count(pcnt_unit, &count);
        pcnt_unit_clear_count(pcnt_unit);

        // pulses/sec = count / period
        float speed = (float)count / TASK_PERIOD_SEC;
        // RPM = (pulses/sec * 60) / pulses_per_rev
        float rpm = (speed * 60.0f) / (float)PULSES_PER_REV;

        motor_speed = speed;
        motor_rpm   = rpm;

        taskA_total += (esp_timer_get_time() - t_start);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ============================================================
// Task B: PID Controller (10 ms period)
//   Computes PID output and updates LEDC duty.
// ============================================================

void pid_control_task(void *args) {
    uint64_t t_start;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_MS);

    for (;;) {
        t_start = esp_timer_get_time();

        float current_rpm = motor_rpm;
        float error = TARGET_RPM - current_rpm;

        // Derivative (change in error per period)
        float derivative_error = error - prev_error;

        // Integral (accumulated error)
        integral_error += error;

        // Anti-windup: clamp integral
        float integral_max = 255.0f / (Ki > 0.0001f ? Ki : 0.0001f);
        if (integral_error >  integral_max) integral_error =  integral_max;
        if (integral_error < -integral_max) integral_error = -integral_max;

        // PID output
        pwm_output += (Kp * error) + (Ki * integral_error) + (Kd * derivative_error);

        // Clamp to valid duty range
        if (pwm_output > 255.0f) pwm_output = 255.0f;
        if (pwm_output < 0.0f)   pwm_output = 0.0f;

        set_duty((uint32_t)pwm_output);

        prev_error = error;

        taskB_total += (esp_timer_get_time() - t_start);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
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
    Serial.println("=== Motor Control Lab ===");
    Serial.printf("Target RPM: %.0f\n", TARGET_RPM);
    Serial.printf("PID gains: Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
    Serial.printf("Task period: %d ms\n", TASK_PERIOD_MS);

    // Direction = 0, Brake = 0
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_BRAKE, OUTPUT);
    digitalWrite(PIN_DIR, LOW);
    digitalWrite(PIN_BRAKE, LOW);
    Serial.println("DIR=LOW, BRAKE=LOW");

    setup_LEDC();
    Serial.println("LEDC configured (20 kHz, 8-bit, duty=0)");

    setup_PCNT();
    Serial.println("PCNT configured (ENC_A1, rising edge, 100 ppr)");

    start_time = esp_timer_get_time();

    // Create tasks at high priority
    TaskHandle_t taskA_handle, taskB_handle;

    xTaskCreate(speed_monitor_task, "speed_mon",
                1 << 16, NULL, configMAX_PRIORITIES - 1, &taskA_handle);
    Serial.println("Speed monitor task created");

    xTaskCreate(pid_control_task, "pid_ctrl",
                1 << 16, NULL, configMAX_PRIORITIES - 2, &taskB_handle);
    Serial.println("PID control task created");

    Serial.println("=== Running ===");
    Serial.flush();
}

void loop() {
    delay(1000);

    uint64_t elapsed = esp_timer_get_time() - start_time;
    float util_A = 0, util_B = 0;
    if (elapsed > 0) {
        util_A = 100.0f * (float)taskA_total / (float)elapsed;
        util_B = 100.0f * (float)taskB_total / (float)elapsed;
    }

    Serial.printf("RPM: %7.1f  duty: %5.1f  err: %7.1f  "
                  "CPU: A=%.2f%% B=%.2f%%\n",
                  motor_rpm, pwm_output,
                  TARGET_RPM - motor_rpm,
                  util_A, util_B);
}
