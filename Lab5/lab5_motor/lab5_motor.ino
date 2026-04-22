// Lab 5: Motor Control
// ESP32-S3 — ESP-IDF LEDC PWM, PCNT encoder, PID control
//
// Pin assignment (Motor 1):
//   GPIO 5  -> PWM1    (LEDC, inverted: hw_duty = 255 - logical_duty)
//   GPIO 4  -> DIR1    (held LOW)
//   GPIO 21 -> BRAKE   (held HIGH to release)
//   GPIO 6  -> ENC_A1  (PCNT, 100 ppr, rising edge)
//
// IDE: Tools -> "USB CDC On Boot" -> Enabled

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

// ============================================================
// Pin definitions
// ============================================================

#define PIN_PWM     5
#define PIN_DIR     4
#define PIN_BRAKE   21
#define PIN_ENC_A   6

// ============================================================
// Motor / PID parameters
// ============================================================

#define MOTOR_PWM_FREQ      20000
#define PULSES_PER_REV      100

#define TASK_PERIOD_MS      10
#define TASK_PERIOD_SEC     (TASK_PERIOD_MS / 1000.0f)

#define TARGET_RPM          1500.0f

// Tuned gains — conservative to avoid oscillation
static float Kp = 0.05f;
static float Ki = 0.005f;
static float Kd = 0.0f;

// ============================================================
// Shared state
// ============================================================

static volatile float motor_rpm  = 0;
static volatile float pwm_output = 0;

static float integral_error = 0;
static float prev_error     = 0;

static volatile uint64_t taskA_total = 0;
static volatile uint64_t taskB_total = 0;
static uint64_t start_time = 0;

static pcnt_unit_handle_t    pcnt_unit    = NULL;
static pcnt_channel_handle_t pcnt_channel = NULL;

// ============================================================
// LEDC setup
// ============================================================

void setup_LEDC() {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = MOTOR_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = PIN_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 255,
        .hpoint     = 0
    };
    ledc_channel_config(&channel);
}

// Logical duty: 0=stopped, 255=full speed
// Hardware is inverted
static void set_duty(uint32_t duty) {
    if (duty > 255) duty = 255;
    uint32_t hw_duty = 255 - duty;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, hw_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ============================================================
// PCNT setup
// ============================================================

void setup_PCNT() {
    pcnt_unit_config_t unit_config = {
        .low_limit     = -1,
        .high_limit    = (1 << 14),
        .intr_priority = 0,
        .flags         = { .accum_count = 0 }
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
// Task A: Speed Monitor (10 ms)
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

        float speed_pps = (float)count / TASK_PERIOD_SEC;
        motor_rpm = (speed_pps * 60.0f) / (float)PULSES_PER_REV;

        taskA_total += (esp_timer_get_time() - t_start);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ============================================================
// Task B: PID Controller (10 ms)
//
// Direct PID (not incremental):
//   output = Kp*error + Ki*integral + Kd*derivative
// ============================================================

void pid_control_task(void *args) {
    uint64_t t_start;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_MS);

    for (;;) {
        t_start = esp_timer_get_time();

        float error            = TARGET_RPM - motor_rpm;
        float derivative_error = error - prev_error;
        integral_error        += error;

        // Anti-windup: clamp integral
        float integral_max = 255.0f / (Ki > 0.0001f ? Ki : 0.0001f);
        if (integral_error >  integral_max) integral_error =  integral_max;
        if (integral_error < -integral_max) integral_error = -integral_max;

        // Direct PID output (not +=)
        pwm_output = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error);

        // Clamp
        if (pwm_output > 255.0f) pwm_output = 255.0f;
        if (pwm_output <   0.0f) pwm_output =   0.0f;

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

    pinMode(PIN_DIR,   OUTPUT); digitalWrite(PIN_DIR,   LOW);
    pinMode(PIN_BRAKE, OUTPUT); digitalWrite(PIN_BRAKE, HIGH);
    Serial.println("DIR=LOW, BRAKE=HIGH (released)");

    setup_LEDC();
    Serial.println("LEDC configured (20 kHz, 8-bit, stopped)");

    setup_PCNT();
    Serial.println("PCNT configured (ENC_A1, rising edge, 100 ppr)");

    start_time = esp_timer_get_time();

    TaskHandle_t taskA_handle, taskB_handle;
    xTaskCreate(speed_monitor_task, "speed_mon",
                1 << 16, NULL, configMAX_PRIORITIES - 1, &taskA_handle);
    xTaskCreate(pid_control_task,   "pid_ctrl",
                1 << 16, NULL, configMAX_PRIORITIES - 2, &taskB_handle);

    Serial.println("Tasks created. Entering loop.");
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