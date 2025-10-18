/*
  Narrowband Lambda Sensor Simulator with Combustion Model

  This sketch simulates the output of a narrowband O2 sensor by first modeling
  the basic physics of an internal combustion engine. It calculates the Air-Fuel
  Ratio (AFR) based on configurable engine parameters and dynamic inputs, and then
  generates the corresponding lean/rich voltage signal.

  This approach allows for more realistic testing of engine controllers, as the
  lambda signal is a direct result of a simulated combustion process.

  --------------------
  How it Works:
  --------------------
  1.  **Configuration:** Engine parameters like displacement, number of cylinders,
      and ambient conditions are set as constants at the top of the file.
  2.  **Dynamic Inputs:** Two potentiometers are used to simulate real-time
      changes in engine operation:
      -   Injector Pulse Width (ms): How long the fuel injectors are open.
      -   Manifold Absolute Pressure (kPa): Represents engine load.
  3.  **Physics Model:**
      -   The code calculates the mass of air entering a cylinder based on the MAP
        and ambient conditions (Ideal Gas Law).
      -   It calculates the mass of fuel injected based on the pulse width, using
        a lookup table.
      -   It computes the AFR (Air Mass / Fuel Mass).
      -   It converts AFR to Lambda (AFR / Stoichiometric AFR).
  4.  **Output:** Based on the calculated Lambda value, it generates a sharp
      voltage switch (~0.1V for lean, ~0.9V for rich) on a DAC pin, mimicking
      a real narrowband sensor.
*/

#include <driver/dac.h>    // ESP32 DAC driver
#include <math.h>          // For mathematical functions

//================================================================================
// --- SIMULATOR CONFIGURATION ---
//================================================================================

// --- 1. Engine Parameters ---
const float ENGINE_DISPLACEMENT_L = 2.0f;   // Total engine displacement in Liters
const int   NUM_CYLINDERS = 4;              // Number of engine cylinders

// --- 2. Ambient Conditions ---
// These are used for the air mass calculation.
const float AIR_TEMPERATURE_C = 20.0f;      // Ambient air temperature in Celsius
const float RELATIVE_HUMIDITY_PERCENT = 50.0f; // Relative humidity in percent

// --- 3. Dynamic Input Pins (ESP32) ---
const int POT_INJ_PULSE_PIN = 32; // Potentiometer to control injector pulse width
const int POT_MAP_PIN = 33;       // Potentiometer to control manifold pressure

// --- 4. Output Pin (ESP32) ---
const int LAMBDA_OUT_PIN = 25;    // DAC1 (GPIO25) for the simulated lambda signal

// --- 5. Simulation Control ---
const float MIN_INJ_PULSE_MS = 1.0f;  // Min injector pulse width for pot control (ms)
const float MAX_INJ_PULSE_MS = 15.0f; // Max injector pulse width for pot control (ms)
const float MIN_MAP_KPA = 20.0f;      // Min manifold pressure for pot control (kPa, idle)
const float MAX_MAP_KPA = 100.0f;     // Max manifold pressure for pot control (kPa, full load)

//================================================================================
// --- PHYSICAL & FUEL CONSTANTS ---
//================================================================================
const float STOICHIOMETRIC_AFR = 14.7f; // Stoichiometric AFR for gasoline
const float RD_GAS_CONSTANT = 287.05f;  // Specific gas constant for dry air (J/kg*K)
const float RV_GAS_CONSTANT = 461.5f;   // Specific gas constant for water vapor (J/kg*K)

// Fuel Injector Flow Lookup Table
// Maps injector pulse width (ms) to the mass of fuel injected (mg)
const int   FUEL_TABLE_POINTS = 16;
const float pulseLengthTable_ms[FUEL_TABLE_POINTS] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5};
const float fuelMassTable_mg[FUEL_TABLE_POINTS]    = {0.0, 2.0, 4.5, 7.0, 10.0, 13.5, 17.0, 21.0, 25.0, 29.5, 34.0, 39.0, 44.0, 49.5, 55.0, 61.0};

// --- Function Prototypes ---
float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L);
float interpolateFuelMass_mg(float pulseLength_ms);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
  Serial.begin(115200);
  Serial.println("Combustion-Model Lambda Simulator (Narrowband) Initialized.");

  // Configure ADC inputs
  analogReadResolution(12); // Set ADC to 12-bit resolution (0-4095)

  // Start the DAC output
  dac_output_enable(DAC_CHANNEL_1);
}

void loop() {
  // --- Read Dynamic Inputs from Potentiometers ---
  int pot_inj_val = analogRead(POT_INJ_PULSE_PIN);
  int pot_map_val = analogRead(POT_MAP_PIN);

  // Map potentiometer readings to physical values
  float injPulseLength_ms = mapFloat(pot_inj_val, 0, 4095, MIN_INJ_PULSE_MS, MAX_INJ_PULSE_MS);
  float map_kPa = mapFloat(pot_map_val, 0, 4095, MIN_MAP_KPA, MAX_MAP_KPA);

  // --- Run the Combustion Simulation ---
  float singleCylinderVolume_L = ENGINE_DISPLACEMENT_L / NUM_CYLINDERS;

  // 1. Calculate the mass of air drawn into a single cylinder for one cycle
  float airMass_mg = calculateAirMass_mg(map_kPa, AIR_TEMPERATURE_C, RELATIVE_HUMIDITY_PERCENT, singleCylinderVolume_L);

  // 2. Calculate the mass of fuel injected for one cycle
  float fuelMass_mg = interpolateFuelMass_mg(injPulseLength_ms);

  // 3. Calculate Air-Fuel Ratio and Lambda
  float AFR = 0;
  if (fuelMass_mg > 0.01) { // Avoid division by zero
    AFR = airMass_mg / fuelMass_mg;
  }
  float lambda = AFR / STOICHIOMETRIC_AFR;

  // --- Generate Sensor Output ---
  // A real narrowband sensor switches sharply at Lambda = 1.0
  const float LEAN_VOLTAGE = 0.1f;
  const float RICH_VOLTAGE = 0.9f;
  float outputVoltage = (lambda >= 1.0) ? LEAN_VOLTAGE : RICH_VOLTAGE;

  // Convert voltage to 8-bit DAC value (0-255) and output it
  // ESP32 DAC is 0-3.3V, so our 0.1-0.9V range is well within limits.
  int dacValue = (outputVoltage / 3.3f) * 255;
  dac_output_voltage(DAC_CHANNEL_1, dacValue);

  // --- Debugging Output ---
  Serial.print("MAP: "); Serial.print(map_kPa, 1); Serial.print(" kPa");
  Serial.print(" | Inj Pulse: "); Serial.print(injPulseLength_ms, 2); Serial.print(" ms");
  Serial.print(" | Air: "); Serial.print(airMass_mg, 1); Serial.print(" mg");
  Serial.print(" | Fuel: "); Serial.print(fuelMass_mg, 1); Serial.print(" mg");
  Serial.print(" | AFR: "); Serial.print(AFR, 1);
  Serial.print(" | Lambda: "); Serial.print(lambda, 2);
  Serial.print(" | V_out: "); Serial.println(outputVoltage, 2);

  delay(100);
}

/**
 * @brief Calculates the mass of air in a cylinder using the Ideal Gas Law, adjusted for humidity.
 * @param map_kPa Manifold Absolute Pressure in kilopascals.
 * @param airTemp_C Air temperature in Celsius.
 * @param humidity_percent Relative humidity in percent.
 * @param cylinderVolume_L The volume of a single cylinder in Liters.
 * @return The mass of air in milligrams (mg).
 */
float calculateAirMass_mg(float map_kPa, float airTemp_C, float humidity_percent, float cylinderVolume_L) {
  float map_Pa = map_kPa * 1000.0f; // Convert kPa to Pascals
  float airTemp_K = airTemp_C + 273.15f; // Convert Celsius to Kelvin
  float cylinderVolume_m3 = cylinderVolume_L / 1000.0f; // Convert Liters to cubic meters

  // Calculate saturation vapor pressure (using Magnus-Tetens approximation)
  float saturationVaporPressure_Pa = 610.78f * expf((17.27f * airTemp_C) / (airTemp_C + 237.3f));

  // Calculate partial pressure of water vapor
  float vaporPressure_Pa = (humidity_percent / 100.0f) * saturationVaporPressure_Pa;

  // Calculate partial pressure of dry air
  float dryAirPressure_Pa = map_Pa - vaporPressure_Pa;
  if (dryAirPressure_Pa < 0) dryAirPressure_Pa = 0;

  // Calculate density of dry air and water vapor
  float dryAirDensity_kg_m3 = dryAirPressure_Pa / (RD_GAS_CONSTANT * airTemp_K);
  float vaporDensity_kg_m3 = vaporPressure_Pa / (RV_GAS_CONSTANT * airTemp_K);
  if (vaporDensity_kg_m3 < 0) vaporDensity_kg_m3 = 0;

  // Total density is the sum of the two
  float totalDensity_kg_m3 = dryAirDensity_kg_m3 + vaporDensity_kg_m3;

  // Calculate total mass of air in the cylinder (in kg)
  float airMass_kg = totalDensity_kg_m3 * cylinderVolume_m3;

  // Return mass in milligrams
  return airMass_kg * 1000000.0f;
}

/**
 * @brief Interpolates fuel mass from a lookup table based on injector pulse length.
 * @param pulseLength_ms The injector pulse width in milliseconds.
 * @return The mass of fuel in milligrams (mg).
 */
float interpolateFuelMass_mg(float pulseLength_ms) {
  // Find the two points in the table that the pulse length falls between
  for (int i = 0; i < FUEL_TABLE_POINTS - 1; i++) {
    if (pulseLength_ms >= pulseLengthTable_ms[i] && pulseLength_ms <= pulseLengthTable_ms[i+1]) {
      // Linearly interpolate between the two points
      return mapFloat(pulseLength_ms, pulseLengthTable_ms[i], pulseLengthTable_ms[i+1], fuelMassTable_mg[i], fuelMassTable_mg[i+1]);
    }
  }
  // If outside the table, return the boundary value
  if (pulseLength_ms < pulseLengthTable_ms[0]) return fuelMassTable_mg[0];
  if (pulseLength_ms > pulseLengthTable_ms[FUEL_TABLE_POINTS - 1]) return fuelMassTable_mg[FUEL_TABLE_POINTS - 1];
  return 0.0; // Should not be reached
}

/**
 * @brief Maps a float value from one range to another.
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
