/*
  Narrowband Lambda Sensor Simulator with Advanced Engine Dynamics

  This sketch simulates the output of a narrowband O2 sensor by first modeling
  the basic physics of an internal combustion engine.

  --------------------
  New Features in this version:
  --------------------
  1.  **Sensor Warm-up Simulation:** On startup, the sensor stays inactive (held at 0.45V)
      for a configurable period, mimicking the heating time of a real sensor.
  2.  **Manifold Dynamics:** Improved input filtering to specifically model the
      pressure lag in the intake manifold.
  3.  **Transport Delay:** Realistic delay from cylinder to sensor.
  4.  **Sigmoid Output:** High-fidelity narrowband signal curve.
*/

#include <driver/dac.h>
#include <math.h>

//================================================================================
// --- SIMULATOR CONFIGURATION ---
//================================================================================

// --- 1. Engine Parameters ---
const float ENGINE_DISPLACEMENT_L = 2.0f;
const int   NUM_CYLINDERS = 4;

// --- 2. Ambient Conditions ---
const float AIR_TEMPERATURE_C = 20.0f;
const float RELATIVE_HUMIDITY_PERCENT = 50.0f;

// --- 3. Dynamic Input Pins (ESP32) ---
const int POT_INJ_PULSE_PIN = 32;
const int POT_MAP_PIN = 33;

// --- 4. Output Pin (ESP32) ---
const int LAMBDA_OUT_PIN = 25;

// --- 5. Simulation Control ---
const float MIN_INJ_PULSE_MS = 1.0f;
const float MAX_INJ_PULSE_MS = 15.0f;
const float MIN_MAP_KPA = 20.0f;
const float MAX_MAP_KPA = 100.0f;

// --- 6. Sensor Warm-up ---
const unsigned long WARMUP_TIME_MS = 5000;  // 5 seconds heater warm-up time

// --- 7. Injector & Manifold Dynamics ---
const float INJ_FLOW_MG_MS = 8.5f;
const float INJ_DEAD_TIME_MS = 0.8f;
const float MANIFOLD_EMA_ALPHA = 0.15f;     // Manifold filling dynamics
const float INJECTOR_EMA_ALPHA = 0.35f;     // Injector response dynamics
const int   DELAY_BUFFER_SIZE = 10;

//================================================================================
// --- PHYSICAL CONSTANTS ---
//================================================================================
const float STOICHIOMETRIC_AFR = 14.7f;
const float RD_GAS_CONSTANT = 287.05f;
const float RV_GAS_CONSTANT = 461.5f;

// --- Global State ---
float filteredInjVal = 2048.0f;
float filteredMapVal = 2048.0f;
float lambdaBuffer[DELAY_BUFFER_SIZE];
int   bufferIndex = 0;
unsigned long startTime_ms;

// --- Prototypes ---
float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L);
float calculateFuelMass_mg(float pulseLength_ms);
float getNarrowbandVoltage(float lambda);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
  Serial.begin(115200);
  Serial.println("Narrowband Lambda Simulator (Advanced Dynamics) Initialized.");

  analogReadResolution(12);
  dac_output_enable(DAC_CHANNEL_1);

  for (int i = 0; i < DELAY_BUFFER_SIZE; i++) lambdaBuffer[i] = 1.0f;
  startTime_ms = millis();
}

void loop() {
  unsigned long currentTime = millis();
  bool isWarmedUp = (currentTime - startTime_ms) >= WARMUP_TIME_MS;

  // 1. Read and Filter Inputs
  filteredInjVal = (INJECTOR_EMA_ALPHA * analogRead(POT_INJ_PULSE_PIN)) + ((1.0f - INJECTOR_EMA_ALPHA) * filteredInjVal);
  filteredMapVal = (MANIFOLD_EMA_ALPHA * analogRead(POT_MAP_PIN)) + ((1.0f - MANIFOLD_EMA_ALPHA) * filteredMapVal);

  float injPulseLength_ms = mapFloat(filteredInjVal, 0, 4095, MIN_INJ_PULSE_MS, MAX_INJ_PULSE_MS);
  float map_kPa = mapFloat(filteredMapVal, 0, 4095, MIN_MAP_KPA, MAX_MAP_KPA);

  // 2. Combustion Model
  float singleCylinderVolume_L = ENGINE_DISPLACEMENT_L / NUM_CYLINDERS;
  float airMass_mg = calculateAirMass_mg(map_kPa, AIR_TEMPERATURE_C, RELATIVE_HUMIDITY_PERCENT, singleCylinderVolume_L);
  float fuelMass_mg = calculateFuelMass_mg(injPulseLength_ms);

  float instLambda = 1.0f;
  if (fuelMass_mg > 0.01f) {
    instLambda = (airMass_mg / fuelMass_mg) / STOICHIOMETRIC_AFR;
  }

  // 3. Transport Delay
  lambdaBuffer[bufferIndex] = instLambda;
  bufferIndex = (bufferIndex + 1) % DELAY_BUFFER_SIZE;
  float delayedLambda = lambdaBuffer[bufferIndex];

  // 4. Output Generation
  float outputVoltage;
  if (!isWarmedUp) {
    outputVoltage = 0.45f; // Neutral voltage during warm-up
  } else {
    outputVoltage = getNarrowbandVoltage(delayedLambda);
  }

  int dacValue = (outputVoltage / 3.3f) * 255;
  dac_output_voltage(DAC_CHANNEL_1, dacValue);

  // 5. Debug
  if (currentTime % 500 < 20) { // Throttle serial output
    Serial.print("Status: "); Serial.print(isWarmedUp ? "READY" : "WARMING");
    Serial.print(" | MAP: "); Serial.print(map_kPa, 1);
    Serial.print(" | Lambda: "); Serial.print(delayedLambda, 2);
    Serial.print(" | V_out: "); Serial.println(outputVoltage, 2);
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
