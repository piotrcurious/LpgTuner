/*
  Trip Computer

  This sketch provides a comprehensive Trip Computer that calculates and tracks
  vehicle performance metrics including instantaneous fuel consumption, distance,
  and trip-average statistics.

  Key Metrics Provided:
  - Liters per hour (L/h): Instantaneous fuel consumption rate.
  - Liters per 100 kilometers (L/100km): Instantaneous fuel economy.
  - Total Distance (km): Cumulative distance traveled during the current trip.
  - Total Fuel Consumed (L): Cumulative fuel used during the current trip.
  - Average Speed (km/h): Trip-average vehicle speed.
  - Average Economy (L/100km): Trip-average fuel consumption per 100km.

  Precision and Stability:
  To ensure high accuracy over long durations, this sketch employs the **Kahan Summation Algorithm**
  for accumulating fuel and distance. This technique significantly reduces floating-point
  rounding errors by maintaining a separate compensation term for small increments.

  Hardware Assumptions:
  - Microcontroller: Designed for ESP32, Arduino, or similar platforms.
  - Engine RPM: Provided by an ignition or crankshaft sensor.
  - Injector Pulse Width (ms): Time the injector is open per cycle.
  - Vehicle Speed (km/h): Provided by a Vehicle Speed Sensor (VSS) or GPS.
  - Engine Type: Standard 4-stroke (one injection per 2 revolutions per cylinder).

  Configuration:
  - INJECTOR_FLOW_RATE_L_PER_MIN: Calibrate this to your specific fuel injectors.
  - NUM_CYLINDERS: Set this to match your engine's cylinder count.
*/

// --- Constants ---
// !!! IMPORTANT !!!
// You must set this value to match your fuel injectors' flow rate.
const float INJECTOR_FLOW_RATE_L_PER_MIN = 0.5; // Liters per minute

// Set this to the number of cylinders in your engine.
const int NUM_CYLINDERS = 4;

// --- Global Variables for Accumulation ---
float totalFuelConsumed_L = 0.0f;
float fuelCompensation = 0.0f; // Compensation term for Kahan summation (fuel)
float totalDistance_km = 0.0f;
float distanceCompensation = 0.0f; // Compensation term for Kahan summation (distance)
unsigned long lastUpdate_ms = 0;
unsigned long tripStartTime_ms = 0;

// --- Function Prototypes ---
float calculateFuelConsumption_LitersPerHour(float injectionLength_ms, int rpm);
float calculateFuelEconomy_LitersPer100km(float injectionLength_ms, int rpm, float vehicleSpeed_kmh);
void accumulateFuel(float consumption_L_h, float deltaTime_s);
void accumulateDistance(float vehicleSpeed_kmh, float deltaTime_s);

void setup() {
  // Initialize Serial communication for debugging and output
  Serial.begin(115200);
  Serial.println("Trip Computer Initialized.");
  Serial.println("--------------------------");

  // Initialize time tracking
  lastUpdate_ms = millis();
  tripStartTime_ms = lastUpdate_ms;
}

void loop() {
  // --- Example Data Acquisition ---
  // In a production environment, these values would be read from sensors via interrupts or ADC.
  int example_rpm = 2500;                 // Example: engine RPM
  float example_injectionLength_ms = 5.0; // Example: 5ms injection pulse width
  float example_vehicleSpeed_kmh = 80.0;  // Example: 80 km/h

  // 1. Calculate Instantaneous Metrics
  float consumption_L_h = calculateFuelConsumption_LitersPerHour(example_injectionLength_ms, example_rpm);
  float economy_L_100km = calculateFuelEconomy_LitersPer100km(example_injectionLength_ms, example_rpm, example_vehicleSpeed_kmh);

  // 2. Update Cumulative Statistics
  unsigned long current_ms = millis();
  float deltaTime_s = (current_ms - lastUpdate_ms) / 1000.0f;

  if (deltaTime_s > 0) {
    accumulateFuel(consumption_L_h, deltaTime_s);
    accumulateDistance(example_vehicleSpeed_kmh, deltaTime_s);
    lastUpdate_ms = current_ms;
  }

  // 3. Calculate Trip Averages
  float tripTime_h = (current_ms - tripStartTime_ms) / 3600000.0f;
  float averageSpeed_kmh = (tripTime_h > 0) ? (totalDistance_km / tripTime_h) : 0;
  float averageEconomy_L_100km = (totalDistance_km > 0) ? (totalFuelConsumed_L / totalDistance_km * 100.0f) : 0;

  // 4. Output Results
  Serial.print("RPM: "); Serial.print(example_rpm);
  Serial.print(" | Inj: "); Serial.print(example_injectionLength_ms, 2); Serial.print("ms");
  Serial.print(" | Speed: "); Serial.print(example_vehicleSpeed_kmh, 1); Serial.println("km/h");

  Serial.print("=> Instant: "); Serial.print(consumption_L_h, 2); Serial.print(" L/h | ");
  Serial.print(economy_L_100km, 2); Serial.println(" L/100km");

  Serial.println("--- Trip Statistics ---");
  Serial.print("   Distance: "); Serial.print(totalDistance_km, 3); Serial.println(" km");
  Serial.print("   Fuel Consumed: "); Serial.print(totalFuelConsumed_L, 6); Serial.println(" L");
  Serial.print("   Avg Speed: "); Serial.print(averageSpeed_kmh, 1); Serial.println(" km/h");
  Serial.print("   Avg Economy: "); Serial.print(averageEconomy_L_100km, 2); Serial.println(" L/100km");
  Serial.println("--------------------------");

  delay(5000); // Update every 5 seconds
}

/**
 * @brief Calculates fuel consumption in Liters per Hour (L/h).
 *
 * Formula:
 * - Cycles per minute per cylinder = RPM / 2 (for 4-stroke)
 * - Total engine cycles per minute = (RPM / 2) * NUM_CYLINDERS
 * - Total injection time per minute = Total cycles * PulseWidth_ms
 * - L/h = (Total injection time [ms/min] / 60000 [ms/min]) * FlowRate [L/min] * 60 [min/h]
 *
 * @param injectionLength_ms The duration of a single injector pulse in milliseconds.
 * @param rpm The engine speed in revolutions per minute.
 * @return The calculated fuel consumption in L/h.
 */
float calculateFuelConsumption_LitersPerHour(float injectionLength_ms, int rpm) {
  if (rpm <= 0) return 0;

  float injections_per_minute = (rpm / 2.0f) * (float)NUM_CYLINDERS;
  float total_injection_duration_ms_per_min = injections_per_minute * injectionLength_ms;
  float duty_cycle_fraction = total_injection_duration_ms_per_min / 60000.0f;

  return duty_cycle_fraction * INJECTOR_FLOW_RATE_L_PER_MIN * 60.0f;
}

/**
 * @brief Calculates fuel economy in Liters per 100 kilometers (L/100km).
 *
 * @param injectionLength_ms The duration of a single injector pulse in milliseconds.
 * @param rpm The engine speed in revolutions per minute.
 * @param vehicleSpeed_kmh The vehicle's current speed in kilometers per hour.
 * @return The calculated fuel economy in L/100km. Returns 0 if speed is zero.
 */
float calculateFuelEconomy_LitersPer100km(float injectionLength_ms, int rpm, float vehicleSpeed_kmh) {
  if (vehicleSpeed_kmh <= 0) return 0;

  float consumption_L_h = calculateFuelConsumption_LitersPerHour(injectionLength_ms, rpm);
  return (consumption_L_h / vehicleSpeed_kmh) * 100.0f;
}

/**
 * @brief Accumulates fuel consumption using the Kahan summation algorithm to maintain precision.
 *
 * @param consumption_L_h Instantaneous fuel consumption in Liters per Hour.
 * @param deltaTime_s Time elapsed since last update in seconds.
 */
void accumulateFuel(float consumption_L_h, float deltaTime_s) {
  if (deltaTime_s <= 0) return;

  float fuelIncrement = (consumption_L_h / 3600.0f) * deltaTime_s;

  // Kahan summation logic:
  float y = fuelIncrement - fuelCompensation;
  float t = totalFuelConsumed_L + y;
  fuelCompensation = (t - totalFuelConsumed_L) - y;
  totalFuelConsumed_L = t;
}

/**
 * @brief Accumulates distance traveled using the Kahan summation algorithm.
 *
 * @param vehicleSpeed_kmh Instantaneous vehicle speed in kilometers per hour.
 * @param deltaTime_s Time elapsed since last update in seconds.
 */
void accumulateDistance(float vehicleSpeed_kmh, float deltaTime_s) {
  if (deltaTime_s <= 0) return;

  float distanceIncrement = (vehicleSpeed_kmh / 3600.0f) * deltaTime_s;

  // Kahan summation logic:
  float y = distanceIncrement - distanceCompensation;
  float t = totalDistance_km + y;
  distanceCompensation = (t - totalDistance_km) - y;
  totalDistance_km = t;
}
