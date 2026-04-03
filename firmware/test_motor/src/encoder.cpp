#include <Arduino.h>
#include <SPI.h>
#include "hw_config.h"

// Use Encoder 1 for this test
static const int CS_PIN   = PIN_ENCODER3_CS;
static const int SCK_PIN  = PIN_ENCODER_SCK;
static const int MISO_PIN = PIN_ENCODER_MISO;
static const int MOSI_PIN = PIN_ENCODER_MOSI;

// Encoder constants
#define MT6835_CPR 2097152  // Counts per revolution (21-bit)
const float DEG_PER_COUNT = 360.0f / MT6835_CPR;

// Multi-turn tracking
int32_t last_raw = 0;
int32_t abs_position = 0;
bool first_read = true;

// For timing`
unsigned long last_print_time = 0;

// Latest readings
int32_t latest_raw = 0;
uint8_t latest_status = 0;

uint8_t compute_crc8(uint32_t angle21, uint8_t status3) {
    uint32_t data24 = ((angle21 & 0x1FFFFF) << 3) | (status3 & 0x07);
    uint8_t crc = 0;

    for (int i = 23; i >= 0; i--) { // MSB first
        uint8_t bit = (data24 >> i) & 0x01;
        uint8_t crc_msb = (crc >> 7) & 0x01;
        crc <<= 1;
        if (crc_msb ^ bit) {
            crc ^= 0x07; // Polynomial
        }
    }
    return crc & 0xFF;
}

int32_t read_encoder() {
  uint8_t data[6];

  // Prepare command
  data[0] = 0xA0;  // ANGLE command
  data[1] = 0x03;  // Register address
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  for (int i = 0; i < 6; i++) {
    data[i] = SPI.transfer(data[i]);
  }

  delayMicroseconds(1);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();


  // Parse angle (21-bit value)
  latest_raw = ((int32_t)data[2] << 13) |
               ((int32_t)data[3] << 5) |
               (data[4] >> 3);

  // Store status bits
  latest_status = data[4] & 0x07;

  uint8_t crc_calc = compute_crc8(latest_raw, latest_status);
  if (crc_calc != data[5]) {
      //Serial.println("CRC FAILED!");
      return -1;
  }

  return latest_raw;
}

void update_position(int32_t raw) {
  if (first_read) {
    last_raw = raw;
    abs_position = raw;
    first_read = false;
    return;
  }

  int32_t half_max = MT6835_CPR / 2;
  int32_t delta = raw - last_raw;

  // Forward wraparound (359° -> 0°)
  if (delta < -half_max) {
    delta += MT6835_CPR;
  }
  // Backward wraparound (0° -> 359°)
  else if (delta > half_max) {
    delta -= MT6835_CPR;
  }

  abs_position += delta;
  last_raw = raw;
}

void print_data() {
  // Check if 1 second has elapsed
  unsigned long current_time = millis();
  if (current_time - last_print_time < 1000) {
    return;  // Not time to print yet
  }
  
  last_print_time = current_time;

  // Calculate values
  float current_angle = latest_raw * DEG_PER_COUNT;
  int32_t revs = abs_position / MT6835_CPR;
  float total_angle = abs_position * DEG_PER_COUNT;

  // Print formatted output
  Serial.print("  ");
  if (current_angle < 100) Serial.print(" ");
  if (current_angle < 10) Serial.print(" ");
  Serial.print(current_angle, 1);
  Serial.print("°  |  ");

  if (revs >= 0) Serial.print(" ");
  Serial.print(revs);
  Serial.print("   |  ");

  // Adjust spacing for total_angle
  if (abs(total_angle) < 10000) Serial.print(" ");
  if (abs(total_angle) < 1000) Serial.print(" ");
  if (abs(total_angle) < 100) Serial.print(" ");
  if (abs(total_angle) < 10) Serial.print(" ");
  if (total_angle >= 0) Serial.print(" ");
  Serial.print(total_angle, 1);
  Serial.print("°  | ");

  if (latest_status == 0) {
    Serial.print("OK");
  } else {
    Serial.print("ERR:");
    if (latest_status & 0x01) Serial.print(" OVERSPD");
    if (latest_status & 0x02) Serial.print(" WEAK");
    if (latest_status & 0x04) Serial.print(" UVOLT");
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("MT6835 Encoder Reader - Pico (PlatformIO) Test");
  Serial.println("---------------------------------------------");

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Map SPI pins to match PCB wiring
  SPI.setSCK(SCK_PIN);
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.begin();

  delay(100);

  Serial.println("Encoder initialized!");
  Serial.println("Rotate the magnet to see readings...");
  Serial.println();
  Serial.println("Current Angle | Revs | Total Angle | Status");
  Serial.println("--------------|------|-------------|--------");

  last_print_time = millis();
}

void loop() {
  int32_t raw = read_encoder();
  update_position(raw);
  print_data();
}