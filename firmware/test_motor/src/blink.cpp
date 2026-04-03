#include <Arduino.h>

// On Pico / Pico W / Pico 2 W, LED_BUILTIN is usually defined correctly by the core.

// If it isn't, you can override it here.

// For many Pico W variants the LED is driven via the wireless chip; the core still exposes LED_BUILTIN.

#ifndef LED_BUILTIN

#define LED_BUILTIN 25

#endif

static uint32_t counter = 0;

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  // Some boards need a moment for USB CDC serial to enumerate.

  delay(1500);

  Serial.println();

  Serial.println("Pico 2 W Blink + Serial Counter starting...");
}

void loop()
{

  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);

  digitalWrite(LED_BUILTIN, LOW);

  delay(500);

  counter++;

  Serial.print("Counter = ");

  Serial.println(counter);
}
