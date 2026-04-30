/*
  Narrowband Lambda Sensor Simulator with Combustion Model

  This sketch simulates the output of a narrowband O2 sensor by first modeling
  the basic physics of an internal combustion engine. It calculates the Air-Fuel
  Ratio (AFR) based on configurable engine parameters and dynamic inputs, and then
  generates the corresponding lean/rich voltage signal.

  --------------------
  Improvements in this version:
  --------------------
  1.  **Input Filtering:** Exponential Moving Average (EMA) on potentiometer inputs
      to smooth out jitter and simulate manifold/injector dynamics.
  2.  **Linear Injector Model:** Uses a linear flow rate and configurable dead time
      for more realistic fuel mass calculation.
  3.  **Transport Delay:** Simulates the time it takes for the exhaust gas to travel
      from the cylinder to the O2 sensor, varying with engine RPM.
  4.  **Sigmoid Output Curve:** Uses a sigmoid function to generate a smooth but
      sharp transition for the narrowband signal, mimicking real sensor behavior.
*/

#include <driver/dac.h>    // ESP32 DAC driver
#include <math.h>          // For mathematical functions

//================================================================================
// --- SIMULATOR CONFIGURATION ---
//================================================================================

// --- 1. Engine Parameters ---
const float ENGINE_DISPLACEMENT_L = 2.0f;
const int   NUM_CYLINDERS = 4;
const float ENGINE_RPM = 2500.0f;           // Simulated fixed RPM for transport delay

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

// --- 6. Injector Model ---
const float INJ_FLOW_MG_MS = 8.5f;          // Linear flow rate (mg of fuel per ms)
const float INJ_DEAD_TIME_MS = 0.8f;        // Injector opening/closing dead time

// --- 7. Signal Smoothing & Dynamics ---
const float EMA_ALPHA = 0.2f;               // Filtering factor (0.0 to 1.0)
const int   DELAY_BUFFER_SIZE = 10;         // Size of the transport delay buffer

//================================================================================
// --- PHYSICAL CONSTANTS ---
//================================================================================
const float STOICHIOMETRIC_AFR = 14.7f;
const float RD_GAS_CONSTANT = 287.05f;
const float RV_GAS_CONSTANT = 461.5f;

// --- Global Variables for Dynamics ---
float filteredInjVal = 2048.0f;
float filteredMapVal = 2048.0f;
float lambdaBuffer[DELAY_BUFFER_SIZE];
int   bufferIndex = 0;

// --- Function Prototypes ---
float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L);
float calculateFuelMass_mg(float pulseLength_ms);
float getNarrowbandVoltage(float lambda);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
  Serial.begin(115200);
  Serial.println("Improved Narrowband Lambda Simulator Initialized.");

  analogReadResolution(12);
  dac_output_enable(DAC_CHANNEL_1);

  // Initialize delay buffer with stoichiometric lambda
  for (int i = 0; i < DELAY_BUFFER_SIZE; i++) {
    lambdaBuffer[i] = 1.0f;
  }
}

void loop() {
  // --- 1. Read and Filter Dynamic Inputs ---
  int raw_inj = analogRead(POT_INJ_PULSE_PIN);
  int raw_map = analogRead(POT_MAP_PIN);

  filteredInjVal = (EMA_ALPHA * raw_inj) + ((1.0f - EMA_ALPHA) * filteredInjVal);
  filteredMapVal = (EMA_ALPHA * raw_map) + ((1.0f - EMA_ALPHA) * filteredMapVal);

  float injPulseLength_ms = mapFloat(filteredInjVal, 0, 4095, MIN_INJ_PULSE_MS, MAX_INJ_PULSE_MS);
  float map_kPa = mapFloat(filteredMapVal, 0, 4095, MIN_MAP_KPA, MAX_MAP_KPA);

  // --- 2. Run Combustion Model ---
  float singleCylinderVolume_L = ENGINE_DISPLACEMENT_L / NUM_CYLINDERS;
  float airMass_mg = calculateAirMass_mg(map_kPa, AIR_TEMPERATURE_C, RELATIVE_HUMIDITY_PERCENT, singleCylinderVolume_L);
  float fuelMass_mg = calculateFuelMass_mg(injPulseLength_ms);

  float instantaneousLambda = 1.0f;
  if (fuelMass_mg > 0.01f) {
    instantaneousLambda = (airMass_mg / fuelMass_mg) / STOICHIOMETRIC_AFR;
  }

  // --- 3. Apply Transport Delay ---
  // Store instantaneous lambda and retrieve the delayed one
  lambdaBuffer[bufferIndex] = instantaneousLambda;
  bufferIndex = (bufferIndex + 1) % DELAY_BUFFER_SIZE;
  float delayedLambda = lambdaBuffer[bufferIndex];

  // --- 4. Generate Sensor Output ---
  float outputVoltage = getNarrowbandVoltage(delayedLambda);
  int dacValue = (outputVoltage / 3.3f) * 255;
  dac_output_voltage(DAC_CHANNEL_1, dacValue);

  // --- Debugging ---
  Serial.print("MAP: "); Serial.print(map_kPa, 1);
  Serial.print(" | Inj: "); Serial.print(injPulseLength_ms, 2);
  Serial.print(" | Lambda (Inst): "); Serial.print(instantaneousLambda, 2);
  Serial.print(" | Lambda (Del): "); Serial.print(delayedLambda, 2);
  Serial.print(" | V_out: "); Serial.println(outputVoltage, 2);

  delay(20); // Faster loop for smoother dynamics
}

float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L) {
  float map_Pa = map_kPa * 1000.0f;
  float airTemp_K = airTemp_C + 273.15f;
  float cylinderVolume_m3 = cylinderVolume_L / 1000.0f;
  float saturationVaporPressure_Pa = 610.78f * expf((17.27f * airTemp_C) / (airTemp_C + 237.3f));
  float vaporPressure_Pa = (humidity_percent / 100.0f) * saturationVaporPressure_Pa;
  float dryAirPressure_Pa = map_Pa - vaporPressure_Pa;
  if (dryAirPressure_Pa < 0) dryAirPressure_Pa = 0;
  float dryAirDensity_kg_m3 = dryAirPressure_Pa / (RD_GAS_CONSTANT * airTemp_K);
  float vaporDensity_kg_m3 = vaporPressure_Pa / (RV_GAS_CONSTANT * airTemp_K);
  if (vaporDensity_kg_m3 < 0) vaporDensity_kg_m3 = 0;
  float totalDensity_kg_m3 = dryAirDensity_kg_m3 + vaporDensity_kg_m3;
  return (totalDensity_kg_m3 * cylinderVolume_m3) * 1000000.0f;
}

float calculateFuelMass_mg(float pulseLength_ms) {
  float effectivePulse_ms = pulseLength_ms - INJ_DEAD_TIME_MS;
  if (effectivePulse_ms < 0) effectivePulse_ms = 0;
  return effectivePulse_ms * INJ_FLOW_MG_MS;
}

/**
 * @brief Sigmoid-based narrowband lambda curve.
 * Transitions from ~0.9V (rich) to ~0.1V (lean) around lambda 1.0.
 */
float getNarrowbandVoltage(float lambda) {
  const float V_RICH = 0.9f;
  const float V_LEAN = 0.1f;
  const float SLOPE_K = 25.0f; // Sharpness of transition

  // Sigmoid: V_lean + (V_rich - V_lean) / (1 + exp(k * (lambda - 1)))
  return V_LEAN + (V_RICH - V_LEAN) / (1.0f + expf(SLOPE_K * (lambda - 1.0f)));
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
