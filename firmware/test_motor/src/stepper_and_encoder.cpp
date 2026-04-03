#include <Arduino.h>
#include <SPI.h>
#include "hw_config.h"

// ----------------- Encoder config -----------------
#define MT6835_CPR 2097152  // 21-bit counts per rev
const float DEG_PER_COUNT = 360.0f / MT6835_CPR;

// Encoder 1 pins (from hw_config.h)
static const int CS_PIN   = PIN_ENCODER2_CS;
static const int SCK_PIN  = PIN_ENCODER_SCK;
static const int MISO_PIN = PIN_ENCODER_MISO;
static const int MOSI_PIN = PIN_ENCODER_MOSI;

int32_t latest_raw   = 0;
uint8_t latest_status = 0;

// ----------------- Stepper / TB6612 pins -----------------
static const int M1_A_POS = PIN_M1_PWM_A_POS;
static const int M1_A_NEG = PIN_M1_PWM_A_NEG;
static const int M1_B_POS = PIN_M1_PWM_B_POS;
static const int M1_B_NEG = PIN_M1_PWM_B_NEG;

static const int MOTOR_EN    = PIN_MOTOR_EN;
static const int MOTOR_PWMAB = PIN_MOTOR_PWMAB;

// ----------------- Stepper state -----------------
const int STEP_SEQUENCE[4][4] = {
  {1, 0, 1, 0},  // 0: A+, B+
  {0, 1, 1, 0},  // 1: A-, B+
  {0, 1, 0, 1},  // 2: A-, B-
  {1, 0, 0, 1}   // 3: A+, B-
};

int currentStepIndex = 0;

// ----------------- Scan storage -----------------
const int STEPS_PER_REV = 201;
const int TOTAL_STEPS   = STEPS_PER_REV * 2;   // 2 revolutions

struct Sample {
  int   stepIndex;    // 0..TOTAL_STEPS-1
  float angleDeg;     // 0..360
  uint8_t status;     // status bits
};

Sample samples[TOTAL_STEPS];

// ----------------- Encoder read -----------------
int32_t read_encoder_raw() {
  uint8_t data[6];

  data[0] = 0xA0;  // ANGLE command
  data[1] = 0x03;  // register address
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

  latest_raw = ((int32_t)data[2] << 13) |
               ((int32_t)data[3] << 5)  |
               (data[4] >> 3);

  latest_status = data[4] & 0x07;

  return latest_raw;
}

float read_encoder_degrees() {
  int32_t raw = read_encoder_raw();
  return raw * DEG_PER_COUNT;
}

// ----------------- Stepper helpers -----------------
void setMotorCoils(int a_pos, int a_neg, int b_pos, int b_neg) {
  digitalWrite(M1_A_POS, a_pos);
  digitalWrite(M1_A_NEG, a_neg);
  digitalWrite(M1_B_POS, b_pos);
  digitalWrite(M1_B_NEG, b_neg);
}

void stepOne(bool clockwise, int stepDelayMs) {
  if (clockwise) {
    currentStepIndex = (currentStepIndex + 1) & 0x03;
  } else {
    currentStepIndex = (currentStepIndex + 3) & 0x03;  // -1 mod 4
  }

  setMotorCoils(
    STEP_SEQUENCE[currentStepIndex][0],
    STEP_SEQUENCE[currentStepIndex][1],
    STEP_SEQUENCE[currentStepIndex][2],
    STEP_SEQUENCE[currentStepIndex][3]
  );

  delay(stepDelayMs);
}

// ----------------- Scan & report -----------------
bool scanDone = false;

const char* statusText(uint8_t s) {
  if (s == 0) return "OK";

  static char buf[32];
  buf[0] = '\0';

  bool first = true;
  if (s & 0x01) { strcat(buf, first ? "OVERSPD" : "|OVERSPD"); first = false; }
  if (s & 0x02) { strcat(buf, first ? "WEAK"    : "|WEAK");    first = false; }
  if (s & 0x04) { strcat(buf, first ? "UVOLT"   : "|UVOLT");   first = false; }

  return buf;
}

void performTwoRevScan() {
  const int STEP_DELAY_MS = 10;  // slow-ish

  Serial.println();
  Serial.println("=== Two-Revolution Scan ===");
  Serial.print("Total steps: ");
  Serial.println(TOTAL_STEPS);
  Serial.println("Rotating clockwise...");

  int countOK    = 0;
  int countWEAK  = 0;
  int countUVOLT = 0;

  for (int i = 0; i < TOTAL_STEPS; i++) {
    stepOne(true, STEP_DELAY_MS);
    delay(3);  // small settle

    float angle = read_encoder_degrees();

    samples[i].stepIndex = i;
    samples[i].angleDeg  = angle;
    samples[i].status    = latest_status;

    // Track simple statistics on status
    if (latest_status == 0) {
      countOK++;
    } else {
      if (latest_status & 0x02) countWEAK++;
      if (latest_status & 0x04) countUVOLT++;
    }

    // Optional progress print every 20 steps
    if ((i % 20) == 0) {
      Serial.print("Step ");
      Serial.print(i);
      Serial.print("  angle=");
      Serial.print(angle, 2);
      Serial.print(" deg  status=0x");
      Serial.println(latest_status, HEX);
    }
  }

  Serial.println();
  Serial.println("Scan complete.");
  Serial.print("OK samples:    "); Serial.println(countOK);
  Serial.print("WEAK samples:  "); Serial.println(countWEAK);
  Serial.print("UVOLT samples: "); Serial.println(countUVOLT);
  Serial.println();

  Serial.println("step,angle_deg,status_hex,status_text");
  for (int i = 0; i < TOTAL_STEPS; i++) {
    Serial.print(samples[i].stepIndex);
    Serial.print(",");
    Serial.print(samples[i].angleDeg, 3);
    Serial.print(",0x");
    Serial.print(samples[i].status, HEX);
    Serial.print(",");
    Serial.println(statusText(samples[i].status));
  }

  Serial.println("=== End of scan data ===");
}

// ----------------- setup / loop -----------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("Pico 2 - MT6835 Magnetic Strength Scan (2 revs)");
  Serial.println("------------------------------------------------");

  // 15 second countdown so you can open Serial Monitor
  for (int i = 15; i > 0; i--) {
    Serial.print("Starting in ");
    Serial.print(i);
    Serial.println(" s...");
    delay(1000);
  }
  Serial.println("Starting now.");

  // Motor pins
  pinMode(M1_A_POS, OUTPUT);
  pinMode(M1_A_NEG, OUTPUT);
  pinMode(M1_B_POS, OUTPUT);
  pinMode(M1_B_NEG, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_PWMAB, OUTPUT);

  digitalWrite(MOTOR_EN, HIGH);
  digitalWrite(MOTOR_PWMAB, HIGH);  // full power for now

  // Encoder pins
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.setSCK(SCK_PIN);
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.begin();

  delay(100);
  Serial.println("Encoder + motor initialized.");
}

void loop() {
  if (!scanDone) {
    performTwoRevScan();
    scanDone = true;
  }

  // After scan, hold last position
  delay(1000);
}
