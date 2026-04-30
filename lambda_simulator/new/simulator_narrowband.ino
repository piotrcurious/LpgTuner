/*
  Advanced Narrowband Lambda Sensor Simulator

  --------------------
  New Features in this version:
  --------------------
  1.  **RPM-Dependent Transport Delay:** Exhaust travel time now scales with RPM.
  2.  **Sensor Aging Simulation:** Optional sluggishness for old sensors.
  3.  **Dynamic RPM Control:** Added a third potentiometer for RPM.
  4.  **Signal Noise:** Subtle Gaussian-like noise on the output voltage.
*/

#include <driver/dac.h>
#include <math.h>
#include <stdlib.h>

//================================================================================
// --- SIMULATOR CONFIGURATION ---
//================================================================================

// --- 1. Engine Parameters ---
const float ENGINE_DISPLACEMENT_L = 2.0f;
const int   NUM_CYLINDERS = 4;

// --- 2. Dynamic Input Pins (ESP32) ---
const int POT_INJ_PULSE_PIN = 32;
const int POT_MAP_PIN = 33;
const int POT_RPM_PIN = 34;       // NEW: Potentiometer for RPM control

// --- 3. Output Configuration ---
const int LAMBDA_OUT_PIN = 25;
const float SENSOR_AGING_FACTOR = 0.4f; // 1.0 = New, 0.1 = Very sluggish
const float NOISE_AMPLITUDE = 0.005f;   // Simulated noise on output (V)

// --- 4. Simulation Control ---
const float MIN_INJ_PULSE_MS = 1.0f;
const float MAX_INJ_PULSE_MS = 15.0f;
const float MIN_MAP_KPA = 20.0f;
const float MAX_MAP_KPA = 100.0f;
const float MIN_RPM = 500.0f;
const float MAX_RPM = 7000.0f;

// --- 5. Warm-up & Dynamics ---
const unsigned long WARMUP_TIME_MS = 5000;
const float INJ_FLOW_MG_MS = 8.5f;
const float INJ_DEAD_TIME_MS = 0.8f;
const float MANIFOLD_EMA_ALPHA = 0.15f;
const float INJECTOR_EMA_ALPHA = 0.35f;
const int   MAX_DELAY_STEPS = 100;      // Max steps in buffer
const int   IDLE_DELAY_STEPS = 50;     // Max delay steps at idle

//================================================================================
// --- PHYSICAL CONSTANTS ---
//================================================================================
const float STOICHIOMETRIC_AFR = 14.7f;
const float RD_GAS_CONSTANT = 287.05f;
const float RV_GAS_CONSTANT = 461.5f;

// --- Global State ---
float filteredInjVal = 2048.0f;
float filteredMapVal = 2048.0f;
float filteredRpmVal = 1000.0f;
float lambdaBuffer[MAX_DELAY_STEPS];
int   bufferIndex = 0;
float agedOutputVoltage = 0.45f;
unsigned long startTime_ms;

// --- Prototypes ---
float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L);
float calculateFuelMass_mg(float pulseLength_ms);
float getNarrowbandVoltage(float lambda);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
  Serial.begin(115200);
  Serial.println("Advanced Narrowband Simulator (RPM-Dynamics) Initialized.");

  analogReadResolution(12);
  dac_output_enable(DAC_CHANNEL_1);

  for (int i = 0; i < MAX_DELAY_STEPS; i++) lambdaBuffer[i] = 1.0f;
  startTime_ms = millis();
}

void loop() {
  unsigned long currentTime = millis();
  bool isWarmedUp = (currentTime - startTime_ms) >= WARMUP_TIME_MS;

  // 1. Read and Filter Inputs
  filteredInjVal = (INJECTOR_EMA_ALPHA * analogRead(POT_INJ_PULSE_PIN)) + ((1.0f - INJECTOR_EMA_ALPHA) * filteredInjVal);
  filteredMapVal = (MANIFOLD_EMA_ALPHA * analogRead(POT_MAP_PIN)) + ((1.0f - MANIFOLD_EMA_ALPHA) * filteredMapVal);
  filteredRpmVal = (0.1f * analogRead(POT_RPM_PIN)) + (0.9f * filteredRpmVal);

  float injPulseLength_ms = mapFloat(filteredInjVal, 0, 4095, MIN_INJ_PULSE_MS, MAX_INJ_PULSE_MS);
  float map_kPa = mapFloat(filteredMapVal, 0, 4095, MIN_MAP_KPA, MAX_MAP_KPA);
  float rpm = mapFloat(filteredRpmVal, 0, 4095, MIN_RPM, MAX_RPM);

  // 2. Combustion Model
  float singleCylinderVolume_L = ENGINE_DISPLACEMENT_L / NUM_CYLINDERS;
  float airMass_mg = calculateAirMass_mg(map_kPa, 20.0f, 50.0f, singleCylinderVolume_L);
  float fuelMass_mg = calculateFuelMass_mg(injPulseLength_ms);

  float instLambda = 1.0f;
  if (fuelMass_mg > 0.01f) {
    instLambda = (airMass_mg / fuelMass_mg) / STOICHIOMETRIC_AFR;
  }

  // 3. RPM-Dependent Transport Delay
  // Delay steps inversely proportional to RPM
  int delaySteps = (int)(IDLE_DELAY_STEPS * (MIN_RPM / rpm));
  if (delaySteps < 1) delaySteps = 1;
  if (delaySteps > IDLE_DELAY_STEPS) delaySteps = IDLE_DELAY_STEPS;

  // Retrieve delayed value BEFORE overwriting oldest buffer slot
  int delayedIndex = (bufferIndex - delaySteps + MAX_DELAY_STEPS) % MAX_DELAY_STEPS;
  float delayedLambda = lambdaBuffer[delayedIndex];

  // Store instantaneous value
  lambdaBuffer[bufferIndex] = instLambda;
  bufferIndex = (bufferIndex + 1) % MAX_DELAY_STEPS;

  // 4. Output Generation
  float targetVoltage;
  if (!isWarmedUp) {
    targetVoltage = 0.45f;
  } else {
    targetVoltage = getNarrowbandVoltage(delayedLambda);
  }

  // Apply Aging (low-pass filter)
  agedOutputVoltage = (SENSOR_AGING_FACTOR * targetVoltage) + ((1.0f - SENSOR_AGING_FACTOR) * agedOutputVoltage);

  // Add some Noise
  float noise = ((float)rand() / (float)RAND_MAX - 0.5f) * NOISE_AMPLITUDE;
  float finalVoltage = agedOutputVoltage + noise;

  dac_output_voltage(DAC_CHANNEL_1, (int)(constrain(finalVoltage, 0, 3.3f) / 3.3f * 255));

  // 5. Debug
  if (currentTime % 500 < 25) {
    Serial.print("RPM: "); Serial.print(rpm, 0);
    Serial.print(" | Delay: "); Serial.print(delaySteps);
    Serial.print(" | Lambda: "); Serial.print(delayedLambda, 2);
    Serial.print(" | V: "); Serial.println(finalVoltage, 2);
  }

  delay(20);
}

float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L) {
  float map_Pa = map_kPa * 1000.0f;
  float airTemp_K = airTemp_C + 273.15f;
  float saturationVaporPressure_Pa = 610.78f * expf((17.27f * airTemp_C) / (airTemp_C + 237.3f));
  float vaporPressure_Pa = (humidity_percent / 100.0f) * saturationVaporPressure_Pa;
  float dryAirPressure_Pa = map_Pa - vaporPressure_Pa;
  if (dryAirPressure_Pa < 0) dryAirPressure_Pa = 0;
  float dryAirDensity_kg_m3 = dryAirPressure_Pa / (RD_GAS_CONSTANT * airTemp_K);
  float vaporDensity_kg_m3 = vaporPressure_Pa / (RV_GAS_CONSTANT * airTemp_K);
  float totalDensity_kg_m3 = dryAirDensity_kg_m3 + (vaporDensity_kg_m3 > 0 ? vaporDensity_kg_m3 : 0);
  return totalDensity_kg_m3 * (cylinderVolume_L / 1000.0f) * 1000000.0f;
}

float calculateFuelMass_mg(float pulseLength_ms) {
  float effectivePulse_ms = pulseLength_ms - INJ_DEAD_TIME_MS;
  return (effectivePulse_ms > 0) ? (effectivePulse_ms * INJ_FLOW_MG_MS) : 0.0f;
}

float getNarrowbandVoltage(float lambda) {
  return 0.1f + 0.8f / (1.0f + expf(25.0f * (lambda - 1.0f)));
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
