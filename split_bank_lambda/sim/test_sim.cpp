#include <iostream>
#include <vector>
#include <cmath>
#include <deque>
#include <iomanip>
#include <algorithm>

// Mock Arduino definitions
#define INPUT 0
#define OUTPUT 1
#define HIGH 1
#define LOW 0

// Simulation constants
const int SIM_STEPS = 2000;
const float DT = 0.01f; // 10ms step

// Global Mock State
struct MockPin {
    int mode = INPUT;
    int state = LOW;
};
MockPin pins[40];
int analogValues[40];
int dacValues[40];

void pinMode(int pin, int mode) { pins[pin].mode = mode; }
void digitalWrite(int pin, int state) { pins[pin].state = state; }
void dacWrite(int pin, int val) { dacValues[pin] = val; }
int analogRead(int pin) { return analogValues[pin]; }
unsigned long _millis = 0;
unsigned long millis() { return _millis; }

// --- Include Sketch Logic ---
// We redefine the defines to match the sketch but in a way we can control them.
#define INJECTION_PULSE_PIN  34
#define LAMBDA1_PIN          35
#define LAMBDA2_PIN          36
#define DAC1_PIN             25
#define DAC2_PIN             26
#define LAMBDA1_CONDITION_PIN  27
#define LAMBDA1_ENABLE_PIN     14
#define LAMBDA2_CONDITION_PIN  12
#define LAMBDA2_ENABLE_PIN     13

#define LEAN_TARGET          0.45f
#define RICH_TARGET          0.65f
#define DAC_MIN              0
#define DAC_MAX              255
#define DAC_MID              128
#define SMC_GAIN             0.5f
#define SMC_BOUNDARY         0.10f
#define ASYMMETRY_THRESHOLD  15.0f
#define FILTER_ALPHA         0.10f
#define UPDATE_INTERVAL      10

// Sketch Globals
float currentOffset1 = DAC_MID;
float currentOffset2 = DAC_MID;
float filteredLambda1 = 0.5f;
float filteredLambda2 = 0.5f;
unsigned long lastUpdate = 0;

float saturation(float s, float boundary) {
  if (s > boundary) return 1.0f;
  if (s < -boundary) return -1.0f;
  return s / boundary;
}

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void loop_sketch() {
  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_INTERVAL) return;
  lastUpdate = (now / UPDATE_INTERVAL) * UPDATE_INTERVAL;

  float rawL1 = analogRead(LAMBDA1_PIN) * (3.3f / 4095.0f);
  float rawL2 = analogRead(LAMBDA2_PIN) * (3.3f / 4095.0f);

  filteredLambda1 = (rawL1 * FILTER_ALPHA) + (filteredLambda1 * (1.0f - FILTER_ALPHA));
  filteredLambda2 = (rawL2 * FILTER_ALPHA) + (filteredLambda2 * (1.0f - FILTER_ALPHA));

  float s1 = LEAN_TARGET - filteredLambda1;
  float s2 = RICH_TARGET - filteredLambda2;

  currentOffset1 += SMC_GAIN * saturation(s1, SMC_BOUNDARY);
  currentOffset2 += SMC_GAIN * saturation(s2, SMC_BOUNDARY);

  currentOffset1 = constrain(currentOffset1, (float)DAC_MIN, (float)DAC_MAX);
  currentOffset2 = constrain(currentOffset2, (float)DAC_MIN, (float)DAC_MAX);

  int out1 = (int)currentOffset1;
  int out2 = (int)currentOffset2;

  dacWrite(DAC1_PIN, out1);
  dacWrite(DAC2_PIN, out2);

  float drift = currentOffset1 - currentOffset2;
  if (std::abs(drift) > ASYMMETRY_THRESHOLD) {
    pinMode(LAMBDA1_CONDITION_PIN, OUTPUT);
    digitalWrite(LAMBDA1_CONDITION_PIN, drift > 0 ? HIGH : LOW);
    pinMode(LAMBDA1_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA1_ENABLE_PIN, HIGH);
  } else {
    pinMode(LAMBDA1_CONDITION_PIN, INPUT);
    pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  }

  bool atLimit = (out1 <= DAC_MIN || out1 >= DAC_MAX || out2 <= DAC_MIN || out2 >= DAC_MAX);
  if (atLimit) {
    pinMode(LAMBDA2_CONDITION_PIN, OUTPUT);
    if (out1 >= DAC_MAX || out2 >= DAC_MAX) {
      digitalWrite(LAMBDA2_CONDITION_PIN, HIGH);
    } else {
      digitalWrite(LAMBDA2_CONDITION_PIN, LOW);
    }
    pinMode(LAMBDA2_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA2_ENABLE_PIN, HIGH);
  } else {
    pinMode(LAMBDA2_CONDITION_PIN, INPUT);
    pinMode(LAMBDA2_ENABLE_PIN, INPUT);
  }
}

// --- Physics Simulation ---

// Sigmoid function to model narrowband lambda probe voltage vs AFR
// Standard stoich is ~14.7. Sensor switches at Lambda 1.0.
// Voltage is High (0.9V) when Rich, Low (0.1V) when Lean.
float afrToVoltage(float afr) {
    float lambda = afr / 14.7f;
    // Simple sigmoid model: V = 0.5 * (1 - tanh(G * (lambda - 1)))
    // Adjusted to fit typical 0.1V - 0.9V range
    return 0.1f + 0.8f * (0.5f * (1.0f - std::tanh(20.0f * (lambda - 1.0f))));
}

struct EngineSubgroup {
    float baseAfr = 14.7f;
    float offsetImbalance = 0.0f; // Systematic error in this bank
    std::deque<float> exhaustPipe; // Delay line for transport delay
    int transportDelaySteps = 15;  // ~150ms delay

    float currentVoltage = 0.5f;

    void update(float dacOffset) {
        // DAC 128 is 0% change.
        // Assume DAC range 0-255 corresponds to +/- 20% fuel change.
        float fuelFactor = 1.0f + (dacOffset - 128.0f) / 128.0f * 0.20f;

        // Systematic imbalance (e.g. dirty injector)
        fuelFactor *= (1.0f + offsetImbalance);

        float actualAfr = baseAfr / fuelFactor;
        exhaustPipe.push_back(actualAfr);

        if (exhaustPipe.size() > transportDelaySteps) {
            float delayedAfr = exhaustPipe.front();
            exhaustPipe.pop_front();
            currentVoltage = afrToVoltage(delayedAfr);
            // Add some noise
            currentVoltage += ((rand() % 100) - 50) / 1000.0f;
        }
    }
};

int main() {
    srand(42);
    EngineSubgroup bank1, bank2;

    // Scenarios:
    // 1. Bank 1 is naturally 5% lean
    bank1.offsetImbalance = -0.05f;
    // 2. Bank 2 is naturally 2% rich
    bank2.offsetImbalance = 0.02f;

    std::cout << "Step,Time,L1_V,L2_V,Off1,Off2,Drift_En,Lim_En" << std::endl;

    for (int step = 0; step < SIM_STEPS; ++step) {
        // 1. Physics update
        bank1.update(dacValues[DAC1_PIN]);
        bank2.update(dacValues[DAC2_PIN]);

        // 2. Update Mock ADCs (12-bit, 3.3V ref)
        analogValues[LAMBDA1_PIN] = (int)(bank1.currentVoltage * 4095.0f / 3.3f);
        analogValues[LAMBDA2_PIN] = (int)(bank2.currentVoltage * 4095.0f / 3.3f);

        // 3. Controller update
        loop_sketch();

        // 4. Log results every 10 steps (100ms)
        if (step % 10 == 0) {
            std::cout << step << ","
                      << step * DT << ","
                      << bank1.currentVoltage << ","
                      << bank2.currentVoltage << ","
                      << dacValues[DAC1_PIN] << ","
                      << dacValues[DAC2_PIN] << ","
                      << pins[LAMBDA1_ENABLE_PIN].state << ","
                      << pins[LAMBDA2_ENABLE_PIN].state << std::endl;
        }

        _millis += 10;
    }

    return 0;
}
