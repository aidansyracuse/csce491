// CSCE491: Systems Engineering
// Bit-Banged I2C Lab - MPU-6050 Accelerometer / Tilt Angle
// Due: Friday, Feb. 20, 2026

#include <math.h>

// ── Pin Definitions ──────────────────────────────────────────────
// Use unambiguous names; undef any board-header conflicts first.
#undef PIN_SCL
#undef PIN_SDA
#undef SCL
#undef SDA

static const int I2C_SCL = 1;   // PCB label: D1
static const int I2C_SDA = 2;   // PCB label: D2

// ── MPU-6050 Addresses / Registers ───────────────────────────────
#define MPU_ADDR       0x68
#define REG_PWR_MGMT   0x6B
#define REG_ACCEL_XOUT 0x3B

// ── Timing ───────────────────────────────────────────────────────
#define I2C_DELAY_US 10

// ─────────────────────────────────────────────────────────────────
//  Open-drain emulation — no ESP-IDF needed
//
//  Driving HIGH:  pinMode(pin, INPUT_PULLUP)
//                 → releases the line AND activates the ~45kΩ pull-up
//  Driving LOW :  pinMode(pin, OUTPUT) + digitalWrite(pin, LOW)
//                 → actively pulls the line to GND
//
//  This works identically for both SCL and SDA.
// ─────────────────────────────────────────────────────────────────

static inline void pin_high(int pin) {
  pinMode(pin, INPUT_PULLUP);           // release + pull-up ON
}

static inline void pin_low(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);                 // actively drive LOW
}

static inline int pin_read(int pin) {
  return digitalRead(pin);
}

// ─────────────────────────────────────────────────────────────────
//  I2C primitives
// ─────────────────────────────────────────────────────────────────

static void scl_high() { pin_high(I2C_SCL); delayMicroseconds(I2C_DELAY_US); }
static void scl_low()  { pin_low(I2C_SCL);  delayMicroseconds(I2C_DELAY_US); }
static void sda_high() { pin_high(I2C_SDA); }
static void sda_low()  { pin_low(I2C_SDA);  }

static void pulse_scl() {
  pin_high(I2C_SCL); delayMicroseconds(I2C_DELAY_US);
  pin_low(I2C_SCL);  delayMicroseconds(I2C_DELAY_US);
}

// START: SDA falls while SCL is HIGH
static void i2c_start() {
  sda_high(); scl_high();
  sda_low();            // START condition
  scl_low();
}

// STOP: SDA rises while SCL is HIGH
static void i2c_stop() {
  sda_low(); scl_high();
  sda_high();           // STOP condition
  delayMicroseconds(I2C_DELAY_US);
}

// Send one byte MSB-first; return true if slave ACKed
static bool i2c_write_byte(uint8_t data, bool verbose = true) {
  for (int i = 7; i >= 0; i--) {
    if ((data >> i) & 0x01) { sda_high(); } else { sda_low(); }
    pulse_scl();
  }
  // Release SDA, then sample ACK on rising SCL
  sda_high();
  pin_high(I2C_SCL);
  delayMicroseconds(I2C_DELAY_US);
  bool ack = (pin_read(I2C_SDA) == LOW);
  pin_low(I2C_SCL);
  delayMicroseconds(I2C_DELAY_US);
  if (!ack && verbose) {
    Serial.printf("  NACK for byte 0x%02X\n", data);
  }
  return ack;
}

// Read one byte; master ACKs unless it's the final byte
static uint8_t i2c_read_byte(bool send_ack) {
  uint8_t data = 0;
  sda_high();   // release SDA so slave can drive it
  for (int i = 7; i >= 0; i--) {
    pin_high(I2C_SCL); delayMicroseconds(I2C_DELAY_US);
    if (pin_read(I2C_SDA) == HIGH) data |= (1 << i);
    pin_low(I2C_SCL);  delayMicroseconds(I2C_DELAY_US);
  }
  if (send_ack) { sda_low(); } else { sda_high(); }
  pulse_scl();
  sda_high();
  return data;
}

// ─────────────────────────────────────────────────────────────────
//  MPU-6050 Register Access
// ─────────────────────────────────────────────────────────────────

static bool mpu_write_reg(uint8_t reg, uint8_t value) {
  i2c_start();
  bool ok  = i2c_write_byte(MPU_ADDR << 1);
  ok      &= i2c_write_byte(reg);
  ok      &= i2c_write_byte(value);
  i2c_stop();
  return ok;
}

static bool mpu_read_regs(uint8_t reg, uint8_t *buf, uint8_t count) {
  // Tx 1: set internal register pointer
  i2c_start();
  bool ok  = i2c_write_byte(MPU_ADDR << 1);
  ok      &= i2c_write_byte(reg);
  i2c_stop();
  // Tx 2: read data
  i2c_start();
  ok &= i2c_write_byte((MPU_ADDR << 1) | 0x01);
  for (uint8_t i = 0; i < count; i++) {
    buf[i] = i2c_read_byte(i < count - 1);
  }
  i2c_stop();
  return ok;
}

// ─────────────────────────────────────────────────────────────────
//  I2C Bus Scanner
// ─────────────────────────────────────────────────────────────────
static void i2c_scan() {
  Serial.println("\n=== I2C Bus Scan ===");
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    i2c_start();
    uint8_t b = addr << 1;
    for (int i = 7; i >= 0; i--) {
      if ((b >> i) & 0x01) { sda_high(); } else { sda_low(); }
      pulse_scl();
    }
    sda_high();
    pin_high(I2C_SCL); delayMicroseconds(I2C_DELAY_US);
    bool acked = (pin_read(I2C_SDA) == LOW);
    pin_low(I2C_SCL);  delayMicroseconds(I2C_DELAY_US);
    i2c_stop();
    delayMicroseconds(500);
    if (acked) { Serial.printf("  Device found at 0x%02X\n", addr); found++; }
  }
  if (found == 0) {
    Serial.println("  No devices found!");
    Serial.println("  -> Confirm SDA=GPIO21, SCL=GPIO22.");
    Serial.println("  -> Confirm VCC=3.3V, GND=GND on MPU-6050.");
    Serial.println("  -> Add 4.7k pull-up resistors (SDA->3.3V, SCL->3.3V) if still failing.");
  } else {
    Serial.printf("  %d device(s) found.\n", found);
    if (found && Serial.available() == 0) {
      Serial.println("  If address != 0x68, update MPU_ADDR in code.");
    }
  }
  Serial.println("===================\n");
}

// ─────────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Release both lines high via INPUT_PULLUP — enables internal pull-ups
  pin_high(I2C_SCL);
  pin_high(I2C_SDA);
  delayMicroseconds(500);

  i2c_scan();

  Serial.println("Initializing MPU-6050...");
  if (mpu_write_reg(REG_PWR_MGMT, 0x00)) {
    Serial.println("MPU-6050 initialized OK.\n");
  } else {
    Serial.println("WARNING: Init NACK. Check wiring.\n");
  }
}

// ─────────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────────
void loop() {
  uint8_t raw[6];
  if (!mpu_read_regs(REG_ACCEL_XOUT, raw, 6)) {
    Serial.println("Read error from MPU-6050.");
    delay(1000);
    return;
  }

  int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);

  float ax = ax_raw / 16384.0f;
  float ay = ay_raw / 16384.0f;
  float az = az_raw / 16384.0f;

  float xy_angle = atan2(ax, ay) * RAD_TO_DEG;
  float xz_angle = atan2(ax, az) * RAD_TO_DEG;
  float yz_angle = atan2(ay, az) * RAD_TO_DEG;

  Serial.printf("XY angle: %0.2f degrees\n", xy_angle);
  Serial.printf("XZ angle: %0.2f degrees\n", xz_angle);
  Serial.printf("YZ angle: %0.2f degrees\n", yz_angle);
  Serial.println("-----------------------------");

  delay(1000);
}
