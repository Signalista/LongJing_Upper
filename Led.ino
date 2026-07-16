#include "Globals.h"

static bool led1State = false;
static uint32_t lastLed1ToggleMs = 0;
static volatile bool led2Active = false;
static volatile uint32_t lastCommunicationMs = 0;

void setupStatusLeds() {
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  led1State = true;
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, LOW);
  lastLed1ToggleMs = millis();
}

void markCommunicationActivity() {
  lastCommunicationMs = millis();
  led2Active = true;
  digitalWrite(PIN_LED2, HIGH);
}

void updateStatusLeds() {
  uint32_t nowMs = millis();

  if (nowMs - lastLed1ToggleMs >= LED1_TOGGLE_INTERVAL_MS) {
    lastLed1ToggleMs = nowMs;
    led1State = !led1State;
    digitalWrite(PIN_LED1, led1State ? HIGH : LOW);
  }

  if (led2Active && nowMs - lastCommunicationMs >= LED2_HOLD_TIME_MS) {
    led2Active = false;
    digitalWrite(PIN_LED2, LOW);
  }
}
