/*
  Wideband & Narrowband Lambda Simulator with Advanced Engine Dynamics

  --------------------
  Features:
  --------------------
  1.  **Sensor Warm-up Simulation:** Dual-channel warm-up simulation.
  2.  **Advanced Manifold Dynamics:** Differentiated filtering for air and fuel.
  3.  **Transport Delay:** Simulated exhaust travel time.
  4.  **High-Fidelity Dual Outputs:** Linear wideband and sigmoid narrowband.
*/

#include <driver/dac.h>
#include <math.h>

//================================================================================
// --- SIMULATOR CONFIGURATION ---
//================================================================================

const float ENGINE_DISPLACEMENT_L = 2.0f;
const int   NUM_CYLINDERS = 4;
const float VOLUMETRIC_EFFICIENCY = 0.85f;
const float COMBUSTION_EFFICIENCY = 0.98f;

const float AIR_TEMPERATURE_C = 20.0f;
const float RELATIVE_HUMIDITY_PERCENT = 50.0f;

const int POT_INJ_PULSE_PIN = 32;
const int POT_MAP_PIN = 33;

const int NARROWBAND_OUT_PIN = 25;
const int WIDEBAND_OUT_PIN = 26;

const float MIN_INJ_PULSE_MS = 1.0f;
const float MAX_INJ_PULSE_MS = 15.0f;
const float MIN_MAP_KPA = 20.0f;
const float MAX_MAP_KPA = 100.0f;

// --- Warm-up & Dynamics ---
const unsigned long WARMUP_TIME_MS = 8000;  // 8 seconds warm-up for wideband sensor
const float INJ_FLOW_MG_MS = 8.5f;
const float INJ_DEAD_TIME_MS = 0.8f;
const float MANIFOLD_EMA_ALPHA = 0.12f;     // Slower manifold dynamics
const float INJECTOR_EMA_ALPHA = 0.40f;     // Faster injector response
const int   DELAY_BUFFER_SIZE = 12;

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
float afrBuffer[DELAY_BUFFER_SIZE];
int   bufferIndex = 0;
unsigned long startTime_ms;

// --- Prototypes ---
float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L, float ve);
float calculateFuelMass_mg(float pulseLength_ms);
float getNarrowbandVoltage(float lambda);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
  Serial.begin(115200);
  Serial.println("Advanced Lambda Simulator (Improved Dynamics) Initialized.");

  analogReadResolution(12);
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);

  for (int i = 0; i < DELAY_BUFFER_SIZE; i++) afrBuffer[i] = STOICHIOMETRIC_AFR;
  startTime_ms = millis();
}

void loop() {
  unsigned long currentTime = millis();
  bool isWarmedUp = (currentTime - startTime_ms) >= WARMUP_TIME_MS;

  // 1. Filtering
  filteredInjVal = (INJECTOR_EMA_ALPHA * analogRead(POT_INJ_PULSE_PIN)) + ((1.0f - INJECTOR_EMA_ALPHA) * filteredInjVal);
  filteredMapVal = (MANIFOLD_EMA_ALPHA * analogRead(POT_MAP_PIN)) + ((1.0f - MANIFOLD_EMA_ALPHA) * filteredMapVal);

  float injPulseLength_ms = mapFloat(filteredInjVal, 0, 4095, MIN_INJ_PULSE_MS, MAX_INJ_PULSE_MS);
  float map_kPa = mapFloat(filteredMapVal, 0, 4095, MIN_MAP_KPA, MAX_MAP_KPA);

  // 2. Combustion Model
  float singleCylinderVolume_L = ENGINE_DISPLACEMENT_L / NUM_CYLINDERS;
  float airMass_mg = calculateAirMass_mg(map_kPa, AIR_TEMPERATURE_C, RELATIVE_HUMIDITY_PERCENT, singleCylinderVolume_L, VOLUMETRIC_EFFICIENCY);
  float fuelMass_mg = calculateFuelMass_mg(injPulseLength_ms);

  float instAFR = STOICHIOMETRIC_AFR;
  if (fuelMass_mg > 0.01f) {
    instAFR = airMass_mg / (fuelMass_mg * COMBUSTION_EFFICIENCY);
  }

  // 3. Delay
  afrBuffer[bufferIndex] = instAFR;
  bufferIndex = (bufferIndex + 1) % DELAY_BUFFER_SIZE;
  float delayedAFR = afrBuffer[bufferIndex];
  float delayedLambda = delayedAFR / STOICHIOMETRIC_AFR;

  // 4. Output Generation
  float nbVoltage, wbVoltageTarget;

  if (!isWarmedUp) {
    nbVoltage = 0.45f;
    wbVoltageTarget = 0.0f; // Many wideband controllers output 0V or a specific "warming" signal
  } else {
    nbVoltage = getNarrowbandVoltage(delayedLambda);
    wbVoltageTarget = mapFloat(delayedAFR, WIDEBAND_MIN_AFR, WIDEBAND_MAX_AFR, WIDEBAND_MIN_VOLTAGE, WIDEBAND_MAX_VOLTAGE);
    wbVoltageTarget = constrain(wbVoltageTarget, 0.0f, 5.0f);
  }

  // Set Narrowband
  dac_output_voltage(DAC_CHANNEL_1, (int)((nbVoltage / 3.3f) * 255));

  // Set Wideband (constrained to ESP32 3.3V)
  float actualWbDacVoltage = constrain(wbVoltageTarget, 0.0f, 3.3f);
  dac_output_voltage(DAC_CHANNEL_2, (int)((actualWbDacVoltage / 3.3f) * 255));

  // 5. Debug
  if (currentTime % 500 < 20) {
    Serial.print("Status: "); Serial.print(isWarmedUp ? "READY" : "WARMING");
    Serial.print(" | AFR (Del): "); Serial.print(delayedAFR, 1);
    Serial.print(" | NB V: "); Serial.print(nbVoltage, 2);
    Serial.print(" | WB V: "); Serial.println(wbVoltageTarget, 2);
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
