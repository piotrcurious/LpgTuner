/*
  Advanced Wideband & Narrowband Lambda Simulator

  --------------------
  New Features in this version:
  --------------------
  1.  **RPM-Dependent Transport Delay:** Exhaust travel time now scales with RPM.
  2.  **Sensor Aging Simulation:** Sluggishness for both O2 outputs.
  3.  **Dynamic RPM Control:** Added a third potentiometer for RPM.
  4.  **Signal Noise:** Subtle noise for realistic ADC behavior in controllers.
*/

#include <driver/dac.h>
#include <math.h>
#include <stdlib.h>

//================================================================================
// --- SIMULATOR CONFIGURATION ---
//================================================================================

const float ENGINE_DISPLACEMENT_L = 2.0f;
const int   NUM_CYLINDERS = 4;
const float VOLUMETRIC_EFFICIENCY = 0.85f;
const float COMBUSTION_EFFICIENCY = 0.98f;

const int POT_INJ_PULSE_PIN = 32;
const int POT_MAP_PIN = 33;
const int POT_RPM_PIN = 34;

const int NARROWBAND_OUT_PIN = 25;
const int WIDEBAND_OUT_PIN = 26;

const float MIN_INJ_PULSE_MS = 1.0f;
const float MAX_INJ_PULSE_MS = 15.0f;
const float MIN_MAP_KPA = 20.0f;
const float MAX_MAP_KPA = 100.0f;
const float MIN_RPM = 500.0f;
const float MAX_RPM = 7000.0f;

// --- Aging & Noise ---
const float SENSOR_AGING_FACTOR = 0.5f;
const float NOISE_AMPLITUDE = 0.005f;

// --- Warm-up & Dynamics ---
const unsigned long WARMUP_TIME_MS = 8000;
const float INJ_FLOW_MG_MS = 8.5f;
const float INJ_DEAD_TIME_MS = 0.8f;
const float MANIFOLD_EMA_ALPHA = 0.12f;
const float INJECTOR_EMA_ALPHA = 0.40f;
const int   MAX_DELAY_STEPS = 100;
const int   IDLE_DELAY_STEPS = 60;

// --- Wideband Scaling ---
const float WIDEBAND_MIN_AFR = 10.0f;
const float WIDEBAND_MAX_AFR = 20.0f;
const float WIDEBAND_MIN_VOLTAGE = 0.0f;
const float WIDEBAND_MAX_VOLTAGE = 5.0f;

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
float afrBuffer[MAX_DELAY_STEPS];
int   bufferIndex = 0;
float agedNbVoltage = 0.45f;
float agedWbVoltage = 0.00f;
unsigned long startTime_ms;

// --- Prototypes ---
float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L, float ve);
float calculateFuelMass_mg(float pulseLength_ms);
float getNarrowbandVoltage(float lambda);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
  Serial.begin(115200);
  Serial.println("Advanced Wideband Simulator (RPM-Dynamics) Initialized.");

  analogReadResolution(12);
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);

  for (int i = 0; i < MAX_DELAY_STEPS; i++) afrBuffer[i] = STOICHIOMETRIC_AFR;
  startTime_ms = millis();
}

void loop() {
  unsigned long currentTime = millis();
  bool isWarmedUp = (currentTime - startTime_ms) >= WARMUP_TIME_MS;

  // 1. Filtering
  filteredInjVal = (INJECTOR_EMA_ALPHA * analogRead(POT_INJ_PULSE_PIN)) + ((1.0f - INJECTOR_EMA_ALPHA) * filteredInjVal);
  filteredMapVal = (MANIFOLD_EMA_ALPHA * analogRead(POT_MAP_PIN)) + ((1.0f - MANIFOLD_EMA_ALPHA) * filteredMapVal);
  filteredRpmVal = (0.15f * analogRead(POT_RPM_PIN)) + (0.85f * filteredRpmVal);

  float injPulseLength_ms = mapFloat(filteredInjVal, 0, 4095, MIN_INJ_PULSE_MS, MAX_INJ_PULSE_MS);
  float map_kPa = mapFloat(filteredMapVal, 0, 4095, MIN_MAP_KPA, MAX_MAP_KPA);
  float rpm = mapFloat(filteredRpmVal, 0, 4095, MIN_RPM, MAX_RPM);

  // 2. Combustion Model
  float singleCylinderVolume_L = ENGINE_DISPLACEMENT_L / NUM_CYLINDERS;
  float airMass_mg = calculateAirMass_mg(map_kPa, 20.0f, 50.0f, singleCylinderVolume_L, VOLUMETRIC_EFFICIENCY);
  float fuelMass_mg = calculateFuelMass_mg(injPulseLength_ms);

  float instAFR = STOICHIOMETRIC_AFR;
  if (fuelMass_mg > 0.01f) {
    instAFR = airMass_mg / (fuelMass_mg * COMBUSTION_EFFICIENCY);
  }

  // 3. RPM-Dependent Transport Delay
  int delaySteps = (int)(IDLE_DELAY_STEPS * (MIN_RPM / rpm));
  if (delaySteps < 1) delaySteps = 1;
  if (delaySteps > IDLE_DELAY_STEPS) delaySteps = IDLE_DELAY_STEPS;

  int delayedIndex = (bufferIndex - delaySteps + MAX_DELAY_STEPS) % MAX_DELAY_STEPS;
  float delayedAFR = afrBuffer[delayedIndex];

  afrBuffer[bufferIndex] = instAFR;
  bufferIndex = (bufferIndex + 1) % MAX_DELAY_STEPS;

  float delayedLambda = delayedAFR / STOICHIOMETRIC_AFR;

  // 4. Output Generation
  float targetNbV, targetWbV;
  if (!isWarmedUp) {
    targetNbV = 0.45f;
    targetWbV = 0.0f;
  } else {
    targetNbV = getNarrowbandVoltage(delayedLambda);
    targetWbV = mapFloat(delayedAFR, WIDEBAND_MIN_AFR, WIDEBAND_MAX_AFR, WIDEBAND_MIN_VOLTAGE, WIDEBAND_MAX_VOLTAGE);
    targetWbV = constrain(targetWbV, 0.0f, 5.0f);
  }

  // Apply Aging
  agedNbVoltage = (SENSOR_AGING_FACTOR * targetNbV) + ((1.0f - SENSOR_AGING_FACTOR) * agedNbVoltage);
  agedWbVoltage = (SENSOR_AGING_FACTOR * targetWbV) + ((1.0f - SENSOR_AGING_FACTOR) * agedWbVoltage);

  // Add Noise
  float noise = ((float)rand() / (float)RAND_MAX - 0.5f) * NOISE_AMPLITUDE;

  dac_output_voltage(DAC_CHANNEL_1, (int)(constrain(agedNbVoltage + noise, 0, 3.3f) / 3.3f * 255));

  float actualWbDacVoltage = constrain(agedWbVoltage + noise, 0.0f, 3.3f);
  dac_output_voltage(DAC_CHANNEL_2, (int)(actualWbDacVoltage / 3.3f * 255));

  // 5. Debug
  if (currentTime % 500 < 25) {
    Serial.print("RPM: "); Serial.print(rpm, 0);
    Serial.print(" | AFR (Del): "); Serial.print(delayedAFR, 1);
    Serial.print(" | NB V: "); Serial.print(agedNbVoltage, 2);
    Serial.print(" | WB V: "); Serial.println(agedWbVoltage, 2);
  }

  delay(20);
}

float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L, float ve) {
  float map_Pa = map_kPa * 1000.0f;
  float airTemp_K = airTemp_C + 273.15f;
  float saturationVaporPressure_Pa = 610.78f * expf((17.27f * airTemp_C) / (airTemp_C + 237.3f));
  float vaporPressure_Pa = (humidity_percent / 100.0f) * saturationVaporPressure_Pa;
  float dryAirPressure_Pa = map_Pa - vaporPressure_Pa;
  if (dryAirPressure_Pa < 0) dryAirPressure_Pa = 0;
  float dryAirDensity_kg_m3 = dryAirPressure_Pa / (RD_GAS_CONSTANT * airTemp_K);
  float vaporDensity_kg_m3 = vaporPressure_Pa / (RV_GAS_CONSTANT * airTemp_K);
  float totalDensity_kg_m3 = dryAirDensity_kg_m3 + (vaporDensity_kg_m3 > 0 ? vaporDensity_kg_m3 : 0);
  return totalDensity_kg_m3 * (cylinderVolume_L / 1000.0f) * ve * 1000000.0f;
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
