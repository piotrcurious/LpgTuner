#include "ControllerLogic.h"

#ifndef ARDUINO
// Simulation environment must provide these mocks
extern int analogRead(int pin);
extern void dacWrite(int pin, int val);
extern void digitalWrite(int pin, int state);
extern void pinMode(int pin, int mode);
extern unsigned long millis();
#define OUTPUT 1
#define INPUT 0
#endif

float currentOffset1 = DAC_MID;
float currentOffset2 = DAC_MID;
float filteredLambda1 = 0.5f;
float filteredLambda2 = 0.5f;
unsigned long lastUpdate = 0;

void runControllerStep() {
  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_INTERVAL) return;
  lastUpdate = (now / UPDATE_INTERVAL) * UPDATE_INTERVAL;

  // 1. Read and filter inputs
  float rawL1 = analogRead(LAMBDA1_PIN) * (3.3f / 4095.0f);
  float rawL2 = analogRead(LAMBDA2_PIN) * (3.3f / 4095.0f);

  filteredLambda1 = (rawL1 * FILTER_ALPHA) + (filteredLambda1 * (1.0f - FILTER_ALPHA));
  filteredLambda2 = (rawL2 * FILTER_ALPHA) + (filteredLambda2 * (1.0f - FILTER_ALPHA));

  // 2. Sliding Mode Control Logic
  float s1 = LEAN_TARGET - filteredLambda1;
  float s2 = RICH_TARGET - filteredLambda2;

  currentOffset1 += SMC_GAIN * saturation(s1, SMC_BOUNDARY);
  currentOffset2 += SMC_GAIN * saturation(s2, SMC_BOUNDARY);

  currentOffset1 = constrain(currentOffset1, (float)DAC_MIN, (float)DAC_MAX);
  currentOffset2 = constrain(currentOffset2, (float)DAC_MIN, (float)DAC_MAX);

  int out1 = (int)currentOffset1;
  int out2 = (int)currentOffset2;

  // 3. Update DAC outputs
  dacWrite(DAC1_PIN, out1);
  dacWrite(DAC2_PIN, out2);

  // 4. Emulation Logic
  float drift = currentOffset1 - currentOffset2;
  if (std::abs(drift) > ASYMMETRY_THRESHOLD) {
    pinMode(LAMBDA1_CONDITION_PIN, OUTPUT);
    digitalWrite(LAMBDA1_CONDITION_PIN, drift > 0 ? 1 : 0);
    pinMode(LAMBDA1_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA1_ENABLE_PIN, 1);
  } else {
    pinMode(LAMBDA1_CONDITION_PIN, INPUT);
    pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  }

  bool atLimit = (out1 <= DAC_MIN || out1 >= DAC_MAX || out2 <= DAC_MIN || out2 >= DAC_MAX);
  if (atLimit) {
    pinMode(LAMBDA2_CONDITION_PIN, OUTPUT);
    if (out1 >= DAC_MAX || out2 >= DAC_MAX) {
      digitalWrite(LAMBDA2_CONDITION_PIN, 1);
    } else {
      digitalWrite(LAMBDA2_CONDITION_PIN, 0);
    }
    pinMode(LAMBDA2_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA2_ENABLE_PIN, 1);
  } else {
    pinMode(LAMBDA2_CONDITION_PIN, INPUT);
    pinMode(LAMBDA2_ENABLE_PIN, INPUT);
  }
}
