#include <Arduino.h>
// hw_config.h pin definitions
#define PIN_M1_PWM_A_POS  13
#define PIN_M1_PWM_A_NEG  12
#define PIN_M1_PWM_B_POS  14
#define PIN_M1_PWM_B_NEG  15
#define PIN_MOTOR_EN      18
#define PIN_MOTOR_PWMAB   19

void printBoardInfo() {
  Serial.println();
  Serial.println("==== Board Info ====");

  // Which core?
#ifdef ARDUINO_ARCH_RP2040
  Serial.println("Core: ARDUINO_ARCH_RP2040 (Philhower RP2040/RP2350 core)");
#endif

#ifdef ARDUINO_ARCH_MBED
  Serial.println("Core: ARDUINO_ARCH_MBED (official Arduino Mbed core)");
#endif

  // Which board macro?
#ifdef ARDUINO_RASPBERRY_PI_PICO
  Serial.println("Board macro: ARDUINO_RASPBERRY_PI_PICO");
#endif
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
  Serial.println("Board macro: ARDUINO_RASPBERRY_PI_PICO_W");
#endif
#ifdef ARDUINO_RASPBERRY_PI_PICO2
  Serial.println("Board macro: ARDUINO_RASPBERRY_PI_PICO2");
#endif
#ifdef ARDUINO_RASPBERRY_PI_PICO2W
  Serial.println("Board macro: ARDUINO_RASPBERRY_PI_PICO2W");
#endif

  // What does LED_BUILTIN actually compile to?
  Serial.print("LED_BUILTIN numeric value: ");
  Serial.println(LED_BUILTIN);

  Serial.println("=====================");
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  const int STARTUP_DELAY_SEC = 5;

  for (int i = STARTUP_DELAY_SEC; i > 0; i--) {
    Serial.print("Starting in ");
    Serial.print(i);
    Serial.println(" seconds...");
    delay(1000);
  }
  Serial.println("TB6612FNG Stepper Motor Test - Pico 2");
  printBoardInfo();  
  // Configure all motor pins as outputs
  pinMode(PIN_M1_PWM_A_POS, OUTPUT);
  pinMode(PIN_M1_PWM_A_NEG, OUTPUT);
  pinMode(PIN_M1_PWM_B_POS, OUTPUT);
  pinMode(PIN_M1_PWM_B_NEG, OUTPUT);
  pinMode(PIN_MOTOR_EN, OUTPUT);
  pinMode(PIN_MOTOR_PWMAB, OUTPUT);
  
  // Enable the motor driver
  digitalWrite(PIN_MOTOR_EN, HIGH);
  digitalWrite(PIN_MOTOR_PWMAB, HIGH);  // Enable PWM (always on for full power)
  
  Serial.println("Motor driver enabled. Starting test...");
  delay(1000);
}

void setMotorCoils(int a_pos, int a_neg, int b_pos, int b_neg) {
  digitalWrite(PIN_M1_PWM_A_POS, a_pos);
  digitalWrite(PIN_M1_PWM_A_NEG, a_neg);
  digitalWrite(PIN_M1_PWM_B_POS, b_pos);
  digitalWrite(PIN_M1_PWM_B_NEG, b_neg);
}

void stepMotor(int steps, int delayMs, bool clockwise) {
  // Full step sequence for bipolar stepper
  const int sequence[4][4] = {
    {1, 0, 1, 0},  // Step 0: A+, B+
    {0, 1, 1, 0},  // Step 1: A-, B+
    {0, 1, 0, 1},  // Step 2: A-, B-
    {1, 0, 0, 1}   // Step 3: A+, B-
  };
  
  for (int i = 0; i < steps; i++) {
    int stepIndex = clockwise ? (i % 4) : (3 - (i % 4));
    
    setMotorCoils(
      sequence[stepIndex][0],
      sequence[stepIndex][1],
      sequence[stepIndex][2],
      sequence[stepIndex][3]
    );
    
    delay(delayMs);
  }
}

void loop() {
  const int STEPS_PER_REVOLUTION = 201;
  Serial.println("Rotating clockwise ~200 steps (1 revolution)...");
  stepMotor(STEPS_PER_REVOLUTION, 10, true);  // 200 steps = 1 full rotation for 1.8° motor
  delay(2000);
  
  Serial.println("Rotating counter-clockwise ~200 steps...");
  stepMotor(STEPS_PER_REVOLUTION, 10, false);
  delay(2000);
  
  Serial.println("Fast rotation test...");
  stepMotor((STEPS_PER_REVOLUTION*2), 5, true);  // 2 rotations, faster
  delay(2000);
}