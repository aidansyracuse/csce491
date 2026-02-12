#include <Arduino.h>
#include <math.h>

// --- Pin Definitions ---
// Lab text mentions pins 21 and 22 
#define SCL_PIN 22
#define SDA_PIN 21

// --- MPU6050 Constants ---
#define MPU_ADDR 0x68       // Device address 
#define PWR_MGMT_1 0x6B     // Power Management Register [cite: 51]
#define ACCEL_XOUT_H 0x3B   // Start of accel data [cite: 50]

// --- Timing ---
// Lab suggests 2.5us/bit (400khz) but also shows delayMicroseconds(10) in examples [cite: 6, 63]
#define I2C_DELAY 5 

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C pins
  pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN); // [cite: 60]
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN); // [cite: 71]
  
  // Idle state: Both high
  digitalWrite(SCL_PIN, HIGH);
  digitalWrite(SDA_PIN, HIGH);

  delay(100);

  // Wake up MPU6050: Write 0 to register 0x6B [cite: 53]
  Serial.println("Waking up MPU6050...");
  i2c_start();
  if(i2c_write_byte((MPU_ADDR << 1) | 0)) { // Address + Write (0)
     i2c_write_byte(PWR_MGMT_1);            // Register 0x6B [cite: 29]
     i2c_write_byte(0);                     // Value 0 to wake up [cite: 53]
  } else {
    Serial.println("MPU6050 not found!");
  }
  i2c_stop();
}

void loop() {
  int16_t ax, ay, az;
  
  // Transaction to read accel data: 
  // 1. Write register address (0x3B)
  i2c_start();
  i2c_write_byte((MPU_ADDR << 1) | 0); // Write mode
  i2c_write_byte(ACCEL_XOUT_H);        // Target register 0x3B
  i2c_stop(); // Some devices prefer a Repeated Start here, but lab specifies stop then new read [cite: 22]

  // 2. Read 6 bytes starting from 0x3B
  i2c_start();
  i2c_write_byte((MPU_ADDR << 1) | 1); // Read mode (R/W bit = 1) [cite: 38]
  
  // Read X (High, Low), Y (High, Low), Z (High, Low)
  // Master sends ACK for all bytes except the last one (NACK) [cite: 42]
  uint8_t xh = i2c_read_byte(true);  // ACK
  uint8_t xl = i2c_read_byte(true);  // ACK
  uint8_t yh = i2c_read_byte(true);  // ACK
  uint8_t yl = i2c_read_byte(true);  // ACK
  uint8_t zh = i2c_read_byte(true);  // ACK
  uint8_t zl = i2c_read_byte(false); // NACK (last byte)
  
  i2c_stop();

  // Combine bytes into 16-bit signed integers [cite: 11]
  ax = (xh << 8) | xl;
  ay = (yh << 8) | yl;
  az = (zh << 8) | zl;

  // Calculate angles [cite: 96]
  // atan2 returns radians, convert to degrees
  float angle_xy = atan2(ax, ay) * RAD_TO_DEG;
  float angle_xz = atan2(ax, az) * RAD_TO_DEG;
  float angle_yz = atan2(ay, az) * RAD_TO_DEG;

  // Print formatted output [cite: 109]
  Serial.printf("XY angle: %0.2f, XZ angle: %0.2f, YZ angle: %0.2f\n", angle_xy, angle_xz, angle_yz);

  delay(1000); // Repeat every 1 second [cite: 15]
}

// --- Bit-Banging Functions ---

void i2c_start() {
  // Start bit: pull SDA low while SCL is high 
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(I2C_DELAY);
  digitalWrite(SCL_PIN, LOW);
}

void i2c_stop() {
  // Stop bit: pull SDA high while SCL is high 
  digitalWrite(SDA_PIN, LOW);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(I2C_DELAY);
}

// Returns true if ACK received, false if NACK
bool i2c_write_byte(uint8_t data) {
  // Send 8 bits, MSB first [cite: 125]
  for (int i = 7; i >= 0; i--) {
    // Change SDA while SCL is low [cite: 119]
    digitalWrite(SDA_PIN, (data >> i) & 1);
    
    // Pulse SCL [cite: 61-64]
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(I2C_DELAY);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(I2C_DELAY);
  }

  // Read ACK bit
  pinMode(SDA_PIN, INPUT_PULLUP); // Switch to input [cite: 75]
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY);
  
  bool ack = !digitalRead(SDA_PIN); // Low means ACK
  
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN); // Switch back to output
  return ack;
}

uint8_t i2c_read_byte(bool send_ack) {
  uint8_t data = 0;
  pinMode(SDA_PIN, INPUT_PULLUP); // Switch to input [cite: 75]

  // Read 8 bits
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(I2C_DELAY);
    
    if (digitalRead(SDA_PIN)) {
      data |= (1 << i);
    }
    
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(I2C_DELAY);
  }

  // Send ACK or NACK
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA_PIN, send_ack ? LOW : HIGH); // Low for ACK, High for NACK
  
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY);
  digitalWrite(SCL_PIN, LOW);
  delayMicroseconds(I2C_DELAY);
  
  return data;
}