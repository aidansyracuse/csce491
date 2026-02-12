#include <Arduino.h>
#include <math.h>

// --- Pin Definitions ---
#define SCL_PIN 22
#define SDA_PIN 21

// --- MPU6050 Constants ---
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// --- Timing ---
#define I2C_DELAY 5
#define RAD_TO_DEG 57.29577951308232f

// --- Open-drain helpers (emulate open-drain) ---
static inline void sda_drive_low() {
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
}
static inline void sda_release() {
  // release SDA -> pulled high by pull-up
  pinMode(SDA_PIN, INPUT_PULLUP);
}
static inline int sda_read() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  return digitalRead(SDA_PIN);
}
static inline void scl_drive_low() {
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, LOW);
}
static inline void scl_release() {
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, HIGH); // SCL is normally driven; keep as output
}

// --- I2C primitives ---
void i2c_init() {
  // Idle: SCL high, SDA released
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, HIGH);
  sda_release();
  delay(1);
}

void i2c_start() {
  sda_release();
  scl_release();
  delayMicroseconds(I2C_DELAY);
  sda_drive_low();   // SDA low while SCL high -> START
  delayMicroseconds(I2C_DELAY);
  scl_drive_low();
  delayMicroseconds(I2C_DELAY);
}

void i2c_stop() {
  sda_drive_low();
  delayMicroseconds(I2C_DELAY);
  scl_release();
  delayMicroseconds(I2C_DELAY);
  sda_release();     // SDA high while SCL high -> STOP
  delayMicroseconds(I2C_DELAY);
}

// write byte, returns true if ACK (ACK = 0)
bool i2c_write_byte(uint8_t data) {
  for (int i = 7; i >= 0; --i) {
    if ((data >> i) & 1) sda_release(); else sda_drive_low();
    delayMicroseconds(I2C_DELAY);
    scl_release();
    delayMicroseconds(I2C_DELAY);
    scl_drive_low();
    delayMicroseconds(I2C_DELAY);
  }
  // read ACK
  sda_release(); // let slave drive ACK
  delayMicroseconds(I2C_DELAY);
  scl_release();
  delayMicroseconds(I2C_DELAY);
  bool ack = (sda_read() == 0);
  scl_drive_low();
  delayMicroseconds(I2C_DELAY);
  return ack;
}

// read byte; master_ack==true -> master sends ACK (low), else NACK (release)
uint8_t i2c_read_byte(bool master_ack) {
  uint8_t val = 0;
  sda_release();
  for (int i = 7; i >= 0; --i) {
    delayMicroseconds(I2C_DELAY);
    scl_release();
    delayMicroseconds(I2C_DELAY);
    int bit = sda_read();
    if (bit) val |= (1 << i);
    scl_drive_low();
    delayMicroseconds(I2C_DELAY);
  }
  // send ACK/NACK
  if (master_ack) sda_drive_low(); else sda_release();
  delayMicroseconds(I2C_DELAY);
  scl_release();
  delayMicroseconds(I2C_DELAY);
  scl_drive_low();
  delayMicroseconds(I2C_DELAY);
  sda_release();
  return val;
}

// --- MPU helpers ---
bool mpu_write_reg(uint8_t reg, uint8_t val) {
  i2c_start();
  if (!i2c_write_byte((MPU_ADDR << 1) | 0)) { i2c_stop(); return false; }
  if (!i2c_write_byte(reg)) { i2c_stop(); return false; }
  if (!i2c_write_byte(val)) { i2c_stop(); return false; }
  i2c_stop();
  return true;
}

bool mpu_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
  // write reg
  i2c_start();
  if (!i2c_write_byte((MPU_ADDR << 1) | 0)) { i2c_stop(); return false; }
  if (!i2c_write_byte(reg)) { i2c_stop(); return false; }
  i2c_stop();

  // read len bytes
  i2c_start();
  if (!i2c_write_byte((MPU_ADDR << 1) | 1)) { i2c_stop(); return false; }
  for (size_t i = 0; i < len; ++i) {
    bool ack = (i < (len - 1));
    buf[i] = i2c_read_byte(ack);
  }
  i2c_stop();
  return true;
}

void setup() {
  Serial.begin(115200);
  i2c_init();
  delay(100);

  Serial.println("Waking up MPU6050...");
  if (!mpu_write_reg(PWR_MGMT_1, 0x00)) {
    Serial.println("MPU6050 write failed (no ACK). Check wiring and pull-ups.");
  } else {
    Serial.println("MPU6050 awake.");
  }
}

void loop() {
  uint8_t buf[6];
  if (!mpu_read_regs(ACCEL_XOUT_H, buf, 6)) {
    Serial.println("MPU read failed");
    delay(1000);
    return;
  }

  // combine with correct sign extension
  int16_t ax = (int16_t)((int16_t)buf[0] << 8 | buf[1]);
  int16_t ay = (int16_t)((int16_t)buf[2] << 8 | buf[3]);
  int16_t az = (int16_t)((int16_t)buf[4] << 8 | buf[5]);

  const float SCALE = 1.0f / 16384.0f;
  float fx = ax * SCALE;
  float fy = ay * SCALE;
  float fz = az * SCALE;

  float angle_xy = atan2(fx, fy) * RAD_TO_DEG;
  float angle_xz = atan2(fx, fz) * RAD_TO_DEG;
  float angle_yz = atan2(fy, fz) * RAD_TO_DEG;

  Serial.printf("AX=%6.3fg AY=%6.3fg AZ=%6.3fg\n", fx, fy, fz);
  Serial.printf("XY=%0.2fdeg XZ=%0.2fdeg YZ=%0.2fdeg\n\n", angle_xy, angle_xz, angle_yz);

  delay(1000);
}
