// Simple Lambda Probe Monitor
// This code monitors a narrowband lambda probe and estimates AFR.
// It also measures the oscillation frequency and duty cycle.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

// Define analog pins for lambda probe and injector signals
#ifdef ARDUINO
const int lambdaPin = A0;
const int injectorPin = A1;
#else
const int lambdaPin = 0;
const int injectorPin = 1;
#endif

// Define voltage range for lambda probe signal
const float lambdaMin = 0.1;
const float lambdaMax = 0.9;

// Define ideal stoichiometric ratio for gasoline engines
const float stoichRatio = 14.7;

// Define threshold value for AFR crossing stoichiometric point
const float threshold = 0.05;

// Define filter window size for moving average
const int windowSize = 10;

// Global variables
float lambdaVoltage = 0;
float injectorVoltage = 0;
float AFR = 14.7;

float lambdaFiltered = 0;
float injectorFiltered = 0;
float AFRFiltered = 14.7;

float lambdaWindow[windowSize];
float injectorWindow[windowSize];
float AFRWindow[windowSize];

float AFRPrev = 14.7;

float period = 0;
float dutyCycle = 0;
float frequency = 0;

unsigned long startTime = 0;
unsigned long highTime = 0;

// Function to calculate moving average of a sensor reading using a window array
float movingAverage(float reading, float window[]) {
  for (int i = 0; i < windowSize - 1; i++) {
    window[i] = window[i + 1];
  }
  window[windowSize - 1] = reading;

  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += window[i];
  }
  return sum / windowSize;
}

// Map function for floats
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < windowSize; i++) {
    lambdaWindow[i] = 0;
    injectorWindow[i] = 0;
    AFRWindow[i] = 14.7;
  }
  AFRFiltered = 14.7;
  AFRPrev = 14.7;
  startTime = 0;
  period = 0;
  frequency = 0;
  dutyCycle = 0;
}

void loop() {
  lambdaVoltage = analogRead(lambdaPin) * (5.0 / 1023.0);
  injectorVoltage = analogRead(injectorPin) * (5.0 / 1023.0);

  AFR = mapFloat(lambdaVoltage, lambdaMin, lambdaMax, stoichRatio * (1 + threshold), stoichRatio * (1 - threshold));

  lambdaFiltered = movingAverage(lambdaVoltage, lambdaWindow);
  injectorFiltered = movingAverage(injectorVoltage, injectorWindow);
  AFRFiltered = movingAverage(AFR, AFRWindow);

  bool risingEdge = (AFRPrev < stoichRatio) && (AFRFiltered >= stoichRatio);
  bool fallingEdge = (AFRPrev >= stoichRatio) && (AFRFiltered < stoichRatio);

  if (risingEdge) {
    unsigned long now = millis();
    if (startTime != 0) {
        period = (now - startTime) / 1000.0;
        if (period > 0) {
            frequency = 1.0 / period;
            if (highTime > 0) {
                dutyCycle = (highTime / 1000.0) / period;
            }
        }
    }
    startTime = now;
  }
  else if (fallingEdge) {
    unsigned long now = millis();
    if (startTime != 0) {
        highTime = (now - startTime);
    }
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("AFR: ");
    Serial.print(AFRFiltered, 2);
    Serial.print(" | Freq: ");
    Serial.print(frequency, 2);
    Serial.print(" Hz | Duty: ");
    Serial.print(dutyCycle * 100, 1);
    Serial.println(" %");
    lastPrint = millis();
  }

  AFRPrev = AFRFiltered;
}
