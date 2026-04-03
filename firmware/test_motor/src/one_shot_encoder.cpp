#include <Arduino.h>
#include <SPI.h>
#include "hw_config.h"

// *** CHANGE THIS to test each chip ***
static const int CS_PIN   = PIN_ENCODER2_CS;
static const int SCK_PIN  = PIN_ENCODER_SCK;
static const int MISO_PIN = PIN_ENCODER_MISO;
static const int MOSI_PIN = PIN_ENCODER_MOSI;

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.setSCK(SCK_PIN);
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.begin();
  delay(100);

  Serial.println("MT6835 Raw Frame Dump - with magnet mounted");
  Serial.println("--------------------------------------------");
  Serial.println("N  | Raw Bytes (hex)              | Angle     | OVS | WEAK | CRC");
  Serial.println("---|------------------------------|-----------|-----|------|----");
}

void loop() {
  static int count = 0;
  static unsigned long last_time = 0;

  // Throttle to 10 reads per second so we can read the output
  if (millis() - last_time < 100) return;
  last_time = millis();

  if (count >= 50) return;  // Stop after 50 reads
  count++;

  uint8_t data[6];
  data[0] = 0xA0;
  data[1] = 0x03;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x00;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);
  for (int i = 0; i < 6; i++) data[i] = SPI.transfer(data[i]);
  delayMicroseconds(1);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  int32_t raw = ((int32_t)data[2] << 13) |
                ((int32_t)data[3] << 5)  |
                (data[4] >> 3);
  uint8_t status = data[4] & 0x07;
  float angle = raw * (360.0f / 2097152.0f);
  bool ovs  = status & 0x01;
  bool weak = status & 0x02;

  // Compute CRC
  uint32_t data24 = ((raw & 0x1FFFFF) << 3) | (status & 0x07);
  uint8_t crc = 0;
  for (int i = 23; i >= 0; i--) {
    uint8_t bit = (data24 >> i) & 0x01;
    if ((crc >> 7) ^ bit) crc = (crc << 1) ^ 0x07;
    else crc <<= 1;
  }
  bool crc_ok = (crc == data[5]);

  // Print raw bytes
  Serial.print(count < 10 ? " " : "");
  Serial.print(count);
  Serial.print(" | ");
  for (int i = 0; i < 6; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print("| ");
  Serial.print(angle, 2);
  Serial.print("° | ");
  Serial.print(ovs  ? "YES" : "no ");
  Serial.print(" | ");
  Serial.print(weak ? "YES " : "no  ");
  Serial.print(" | ");
  Serial.println(crc_ok ? "OK" : "FAIL");

  if (count == 1000) {
    Serial.println();
    Serial.println("Done. Check if angle is frozen across all 50 reads.");
  }
}