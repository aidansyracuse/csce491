// Analog-to-Digital Conversion Lab - ESP32 Battery Voltage Monitor
// CSCE491: Systems Engineering  |  Due: Friday, Mar. 13, 2026

#include <math.h>

// ── Pin definitions ───────────────────────────────────────────────
#define ADC_PIN        15   // Battery voltage input (after 10k/33k divider)
#define ADC_ENABLE_PIN 34   // Must be set INPUT to enable the ADC path

// Power supply serial: U2UXD at 9600 baud
#define PSU_RX_PIN     12
#define PSU_TX_PIN     13
#define PSU_BAUD       9600

// ── Voltage / ADC constants ───────────────────────────────────────
#define ADC_MAX        4095
#define ADC_REF_V      3.3f

// Voltage divider: pin34 = Vbatt * 10/(10+33)
// So Vbatt = pin34_voltage * 43/10
#define DIVIDER_SCALE  (10.0f / 43.0f)   // Vbatt  ->  pin34 voltage
#define DIVIDER_INV    (43.0f / 10.0f)   // pin34  ->  Vbatt

// Calibration sweep parameters
#define SWEEP_MAX_V    14.2f
#define SWEEP_STEP_V   0.05f
#define PSU_CHANNEL    1
#define PSU_CURR_LIM   0.5f

// ── Piecewise linear model ────────────────────────────────────────
#define NUM_PIECES  16
#define PIECE_SIZE  256   // (ADC_MAX+1) / NUM_PIECES

float cal_m[NUM_PIECES];
float cal_b[NUM_PIECES];

// ── Calibration data storage ──────────────────────────────────────
#define MAX_CAL_POINTS 300
float cal_x[MAX_CAL_POINTS];
float cal_y[MAX_CAL_POINTS];
int   cal_count = 0;

// ─────────────────────────────────────────────────────────────────
//  Power supply (SCPI) helpers
// ─────────────────────────────────────────────────────────────────

static void psu_cmd(const char* cmd) {
  Serial2.print(cmd);
  Serial2.print("\r\n");
  delay(100);
}

static bool psu_init() {
  Serial2.print("*IDN?\r\n");
  delay(600);
  while (Serial2.available()) Serial2.read();

  psu_cmd("SYST:REM");

  char buf[40];
  snprintf(buf, sizeof(buf), "INST:NSEL %d", PSU_CHANNEL);
  psu_cmd(buf);

  snprintf(buf, sizeof(buf), "CURR %.2f", PSU_CURR_LIM);
  psu_cmd(buf);

  psu_cmd("OUTP:ENAB 1");
  psu_cmd("OUTP 1");

  return true;
}

static void psu_set_voltage(float v) {
  char buf[32];
  snprintf(buf, sizeof(buf), "VOLT %.3f", v);
  psu_cmd(buf);
}

// Release PSU back to local control but LEAVE output on
// so the user can manually adjust voltage in monitoring mode
static void psu_release() {
  psu_cmd("DISP:TEXT:CLE");
  psu_cmd("SYST:LOC");
}

// ─────────────────────────────────────────────────────────────────
//  Simple linear regression: f(x) = mx + b
// ─────────────────────────────────────────────────────────────────
static void fit_line(float *xs, float *ys, int n, float &m_out, float &b_out) {
  if (n == 0) {
    m_out = ADC_REF_V / (float)ADC_MAX;
    b_out = 0.0f;
    return;
  }
  if (n == 1) {
    m_out = ADC_REF_V / (float)ADC_MAX;
    b_out = ys[0] - m_out * xs[0];
    return;
  }

  double sum_x  = 0, sum_y  = 0;
  double sum_xy = 0, sum_x2 = 0;

  for (int i = 0; i < n; i++) {
    sum_x  += xs[i];
    sum_y  += ys[i];
    sum_xy += (double)xs[i] * ys[i];
    sum_x2 += (double)xs[i] * xs[i];
  }

  double denom = (double)n * sum_x2 - sum_x * sum_x;

  if (fabs(denom) < 1e-12) {
    m_out = 0.0f;
    b_out = (float)(sum_y / n);
    return;
  }

  m_out = (float)(((double)n * sum_xy - sum_x * sum_y) / denom);
  b_out = (float)((sum_y * sum_x2    - sum_x * sum_xy) / denom);
}

// ─────────────────────────────────────────────────────────────────
//  Build piecewise model from collected calibration data
// ─────────────────────────────────────────────────────────────────
static void build_model() {
  float px[MAX_CAL_POINTS], py[MAX_CAL_POINTS];

  for (int p = 0; p < NUM_PIECES; p++) {
    int lo  = p * PIECE_SIZE;
    int hi  = (p == NUM_PIECES - 1) ? ADC_MAX : lo + PIECE_SIZE - 1;
    int cnt = 0;

    for (int i = 0; i < cal_count; i++) {
      int xi = (int)cal_x[i];
      if (xi >= lo && xi <= hi) {
        px[cnt] = cal_x[i];
        py[cnt] = cal_y[i];
        cnt++;
      }
    }

    fit_line(px, py, cnt, cal_m[p], cal_b[p]);
  }
}

// ─────────────────────────────────────────────────────────────────
//  Apply calibration: raw ADC reading -> corrected pin34 voltage (V)
// ─────────────────────────────────────────────────────────────────
static float calibrate(int adc_raw) {
  int p = adc_raw / PIECE_SIZE;
  if (p >= NUM_PIECES) p = NUM_PIECES - 1;
  float v = cal_m[p] * (float)adc_raw + cal_b[p];
  if (v < 0.0f) v = 0.0f;
  return v;
}

// ─────────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);   // time to open Serial Monitor before calibration starts

  pinMode(ADC_ENABLE_PIN, INPUT);

  // FIX 1: Set full 0-3.3V ADC range (default is only 0-1.1V)
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  Serial.println("Entering auto-calibration mode, please wait");

  Serial2.begin(PSU_BAUD, SERIAL_8N1, PSU_RX_PIN, PSU_TX_PIN);
  delay(300);

  Serial.print("Taking control of power supply...");
  psu_init();
  Serial.println("success");

  psu_cmd("DISP:TEXT \"Calibrating\"");
  Serial.println("Autocalibrating");

  int total_steps = (int)roundf(SWEEP_MAX_V / SWEEP_STEP_V);
  int last_pct    = 0;

  for (int step = 0; step <= total_steps; step++) {
    float v_set = step * SWEEP_STEP_V;
    psu_set_voltage(v_set);
    delay(200);

    int   adc_raw          = analogRead(ADC_PIN);
    float v_actual_divided = v_set * DIVIDER_SCALE;

    if (cal_count < MAX_CAL_POINTS) {
      cal_x[cal_count] = (float)adc_raw;
      cal_y[cal_count] = v_actual_divided;
      cal_count++;
    }

    int pct = (step * 100) / total_steps;
    for (int m = last_pct + 10; m <= pct && m <= 100; m += 10) {
      Serial.printf("%d%% complete\n", m);
      last_pct = m;
    }
  }
  if (last_pct < 100) Serial.println("100% complete");

  build_model();

  // FIX 2: Return front-panel control WITHOUT disabling output
  // User can now manually adjust voltage on the PSU during monitoring mode
  psu_release();
  Serial2.end();

  Serial.println("Entering monitoring mode");
}

// ─────────────────────────────────────────────────────────────────
//  loop() -- monitoring mode: print readings every second
// ─────────────────────────────────────────────────────────────────
void loop() {
  int adc_raw = analogRead(ADC_PIN);

  float raw_voltage       = adc_raw * (ADC_REF_V / (float)ADC_MAX);
  float corrected_voltage = calibrate(adc_raw);
  float actual_vbatt      = corrected_voltage * DIVIDER_INV;

  Serial.printf(
    "Raw voltage: %.2f V [ADC read as %d], corrected voltage: %.2f V, actual voltage: %.2f V\n",
    raw_voltage, adc_raw, corrected_voltage, actual_vbatt
  );

  delay(1000);
}
