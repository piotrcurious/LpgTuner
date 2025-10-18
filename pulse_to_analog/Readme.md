/*
  Wideband & Narrowband Lambda Simulator with Advanced Combustion Model

  This sketch simulates the output of both a wideband AFR controller and a
  narrowband O2 sensor. It does this by modeling the physics of an internal
  combustion engine to calculate a realistic Air-Fuel Ratio (AFR).

  It is an advanced tool for testing and developing engine management systems that
  may require one or both types of lambda signals.

  --------------------
  How it Works:
  --------------------
  1.  **Configuration:** Key engine and environmental parameters are set as
      constants at the top of the file for easy modification.
  2.  **Dynamic Inputs:** Two potentiometers simulate real-time engine changes:
      -   Injector Pulse Width (ms)
      -   Manifold Absolute Pressure (kPa)
  3.  **Physics Model:**
      -   Calculates air mass based on MAP, temperature, humidity, and volumetric
        efficiency.
      -   Calculates fuel mass based on injector pulse width and fuel density.
      -   Applies a combustion efficiency factor to the final AFR calculation.
  4.  **Dual Output:**
      -   **Wideband:** Generates a linear voltage (e.g., 0-5V) on one DAC pin
        that is directly proportional to the calculated AFR.
      -   **Narrowband:** Generates a sharp switching voltage (~0.1V/0.9V) on a
        second DAC pin based on whether the AFR is lean or rich.
*/
#include <driver/dac.h>    // ESP32 DAC driver
#include <math.h>          // For mathematical functions

//================================================================================
// --- SIMULATOR CONFIGURATION ---
//================================================================================

// --- 1. Engine Parameters ---
const float ENGINE_DISPLACEMENT_L = 2.0f;
const int   NUM_CYLINDERS = 4;
const float VOLUMETRIC_EFFICIENCY = 0.85f; // Typical for a naturally aspirated engine

// --- 2. Ambient Conditions ---
const float AIR_TEMPERATURE_C = 20.0f;
const float RELATIVE_HUMIDITY_PERCENT = 50.0f;

// --- 3. Fuel & Combustion ---
const float FUEL_DENSITY_KG_L = 0.74f;     // Density of gasoline at 20°C (kg/L)
const float COMBUSTION_EFFICIENCY = 0.95f; // Assumes 95% of fuel is burned

// --- 4. Dynamic Input Pins (ESP32) ---
const int POT_INJ_PULSE_PIN = 32; // Potentiometer for injector pulse width
const int POT_MAP_PIN = 33;       // Potentiometer for manifold pressure

// --- 5. Output Pins (ESP32) ---
const int NARROWBAND_OUT_PIN = 25; // DAC1 (GPIO25) for narrowband signal
const int WIDEBAND_OUT_PIN = 26;   // DAC2 (GPIO26) for wideband signal

// --- 6. Simulation Control ---
const float MIN_INJ_PULSE_MS = 1.0f;
const float MAX_INJ_PULSE_MS = 15.0f;
const float MIN_MAP_KPA = 20.0f;
const float MAX_MAP_KPA = 100.0f;

// --- 7. Wideband Output Scaling ---
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

// Fuel Injector Flow Lookup Table (ms to mg)
const int   FUEL_TABLE_POINTS = 16;
const float pulseLengthTable_ms[FUEL_TABLE_POINTS] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5};
const float fuelMassTable_mg[FUEL_TABLE_POINTS]    = {0.0, 2.0, 4.5, 7.0, 10.0, 13.5, 17.0, 21.0, 25.0, 29.5, 34.0, 39.0, 44.0, 49.5, 55.0, 61.0};

// --- Function Prototypes ---
float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L, float ve);
float interpolateFuelMass_mg(float pulseLength_ms);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
  Serial.begin(115200);
  Serial.println("Advanced Combustion Simulator (Wideband + Narrowband) Initialized.");

  // Configure ADC inputs
  analogReadResolution(12);

  // Start DAC outputs
  dac_output_enable(DAC_CHANNEL_1); // Narrowband
  dac_output_enable(DAC_CHANNEL_2); // Wideband
}

void loop() {
  // --- Read Dynamic Inputs ---
  int pot_inj_val = analogRead(POT_INJ_PULSE_PIN);
  int pot_map_val = analogRead(POT_MAP_PIN);

  float injPulseLength_ms = mapFloat(pot_inj_val, 0, 4095, MIN_INJ_PULSE_MS, MAX_INJ_PULSE_MS);
  float map_kPa = mapFloat(pot_map_val, 0, 4095, MIN_MAP_KPA, MAX_MAP_KPA);

  // --- Run Simulation ---
  float singleCylinderVolume_L = ENGINE_DISPLACEMENT_L / NUM_CYLINDERS;
  float airMass_mg = calculateAirMass_mg(map_kPa, AIR_TEMPERATURE_C, RELATIVE_HUMIDITY_PERCENT, singleCylinderVolume_L, VOLUMETRIC_EFFICIENCY);
  float fuelMass_mg = interpolateFuelMass_mg(injPulseLength_ms);

  float AFR = 0;
  if (fuelMass_mg > 0.01) {
    AFR = airMass_mg / (fuelMass_mg * COMBUSTION_EFFICIENCY);
  }
  float lambda = AFR / STOICHIOMETRIC_AFR;

  // --- Generate Outputs ---
  // 1. Narrowband Output
  const float LEAN_VOLTAGE = 0.1f;
  const float RICH_VOLTAGE = 0.9f;
  float narrowbandVoltage = (lambda >= 1.0) ? LEAN_VOLTAGE : RICH_VOLTAGE;
  int narrowbandDac = (narrowbandVoltage / 3.3f) * 255;
  dac_output_voltage(DAC_CHANNEL_1, narrowbandDac);

  // 2. Wideband Output
  float widebandVoltage = mapFloat(AFR, WIDEBAND_MIN_AFR, WIDEBAND_MAX_AFR, WIDEBAND_MIN_VOLTAGE, WIDEBAND_MAX_VOLTAGE);
  widebandVoltage = constrain(widebandVoltage, WIDEBAND_MIN_VOLTAGE, WIDEBAND_MAX_VOLTAGE);
  float actualDacVoltage = constrain(widebandVoltage, 0, 3.3f); // ESP32 DAC limit
  int widebandDac = (actualDacVoltage / 3.3f) * 255;
  dac_output_voltage(DAC_CHANNEL_2, widebandDac);

  // --- Debugging Output ---
  Serial.print("MAP: "); Serial.print(map_kPa, 1);
  Serial.print(" | Inj: "); Serial.print(injPulseLength_ms, 2);
  Serial.print(" | AFR: "); Serial.print(AFR, 1);
  Serial.print(" | NB V: "); Serial.print(narrowbandVoltage, 2);
  Serial.print(" | WB V: "); Serial.println(widebandVoltage, 2);

  delay(100);
}

float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L, float ve) {
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

    // Apply volumetric efficiency to the cylinder volume
    float effectiveVolume_m3 = cylinderVolume_m3 * ve;
    float airMass_kg = totalDensity_kg_m3 * effectiveVolume_m3;

    return airMass_kg * 1000000.0f;
}

float interpolateFuelMass_mg(float pulseLength_ms) {
    for (int i = 0; i < FUEL_TABLE_POINTS - 1; i++) {
        if (pulseLength_ms >= pulseLengthTable_ms[i] && pulseLength_ms <= pulseLengthTable_ms[i+1]) {
            return mapFloat(pulseLength_ms, pulseLengthTable_ms[i], pulseLengthTable_ms[i+1], fuelMassTable_mg[i], fuelMassTable_mg[i+1]);
        }
    }
    if (pulseLength_ms < pulseLengthTable_ms[0]) return fuelMassTable_mg[0];
    if (pulseLength_ms > pulseLengthTable_ms[FUEL_TABLE_POINTS - 1]) return fuelMassTable_mg[FUEL_TABLE_POINTS - 1];
    return 0.0;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
