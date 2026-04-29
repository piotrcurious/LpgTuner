/*
  Fuel Computer

  This sketch calculates fuel consumption based on engine RPM, injector pulse width,
  and vehicle speed. It provides two key metrics:
  - Liters per hour (L/h): Useful for understanding consumption while idling or at steady states.
  - Liters per 100 kilometers (L/100km): The standard measure for fuel economy.

  Hardware Assumptions:
  - The code assumes it's running on a microcontroller (like an Arduino or ESP32).
  - Engine RPM is provided by a sensor connected to the ignition system.
  - Injector pulse width (the duration the injector is open) is measured in milliseconds (ms).
  - Vehicle speed is provided in kilometers per hour (km/h).
  - It assumes a standard 4-stroke engine, where an injection event occurs once every two revolutions.

  Configuration:
  - INJECTOR_FLOW_RATE_L_PER_MIN: This is a critical value that you must calibrate for your specific fuel injectors.
    It represents the volume of fuel that flows through a single injector if it were held open for one minute.
    You can often find this value in the manufacturer's datasheet for your injectors or determine it experimentally.
*/

// --- Constants ---
// !!! IMPORTANT !!!
// You must set this value to match your fuel injectors' flow rate.
// The default value is a placeholder.
const float INJECTOR_FLOW_RATE_L_PER_MIN = 0.5; // Liters per minute

// Set this to the number of cylinders in your engine.
const int NUM_CYLINDERS = 4;

// --- Global Variables for Accumulation ---
float totalFuelConsumed_L = 0.0f;
float kahanCompensation = 0.0f; // Compensation term for Kahan summation
unsigned long lastUpdate_ms = 0;

// --- Function Prototypes ---
float calculateFuelConsumption_LitersPerHour(float injectionLength_ms, int rpm);
float calculateFuelEconomy_LitersPer100km(float injectionLength_ms, int rpm, float vehicleSpeed_kmh);
void accumulateFuel(float consumption_L_h, float deltaTime_s);

void setup() {
  // Initialize Serial communication for debugging and output
  Serial.begin(115200);
  Serial.println("Fuel Computer Initialized.");
  Serial.println("--------------------------");
  lastUpdate_ms = millis();
}

void loop() {
  // --- Example Usage ---
  // In a real application, you would get these values from sensors.
  // For this example, we'll use placeholder values.
  int example_rpm = 2500; // Example: engine RPM
  float example_injectionLength_ms = 5.0; // Example: 5ms injection pulse width
  float example_vehicleSpeed_kmh = 80.0; // Example: 80 km/h

  // Calculate consumption in Liters per Hour
  float consumption_L_h = calculateFuelConsumption_LitersPerHour(example_injectionLength_ms, example_rpm);

  // Calculate fuel economy in Liters per 100 km
  float economy_L_100km = calculateFuelEconomy_LitersPer100km(example_injectionLength_ms, example_rpm, example_vehicleSpeed_kmh);

  // Update cumulative fuel consumption
  unsigned long current_ms = millis();
  float deltaTime_s = (current_ms - lastUpdate_ms) / 1000.0f;
  accumulateFuel(consumption_L_h, deltaTime_s);
  lastUpdate_ms = current_ms;

  // Print the results
  Serial.print("RPM: "); Serial.print(example_rpm);
  Serial.print(" | Injection Length: "); Serial.print(example_injectionLength_ms, 2); Serial.print(" ms");
  Serial.print(" | Speed: "); Serial.print(example_vehicleSpeed_kmh, 1); Serial.println(" km/h");

  Serial.print("=> Fuel Consumption: "); Serial.print(consumption_L_h, 2); Serial.println(" L/h");
  Serial.print("=> Fuel Economy: "); Serial.print(economy_L_100km, 2); Serial.println(" L/100km");
  Serial.print("=> Total Consumed: "); Serial.print(totalFuelConsumed_L, 6); Serial.println(" L");
  Serial.println("--------------------------");


  // In a real scenario, the loop would continuously read sensor data.
  delay(5000); // Wait 5 seconds before the next reading
}

/**
 * @brief Calculates fuel consumption in Liters per Hour (L/h).
 *
 * @param injectionLength_ms The duration of a single injector pulse in milliseconds.
 * @param rpm The engine speed in revolutions per minute.
 * @return The calculated fuel consumption in L/h.
 */
float calculateFuelConsumption_LitersPerHour(float injectionLength_ms, int rpm) {
  if (rpm <= 0) {
    return 0; // Avoid division by zero and handle engine-off case
  }

  // For a 4-stroke engine, each cylinder has one injection event every two revolutions.
  // We calculate the total number of injection events per minute for the entire engine.
  float injections_per_minute = (rpm / 2.0f) * (float)NUM_CYLINDERS;

  // Calculate the total time ALL injectors are open per minute (in milliseconds).
  // This assumes all injectors have the same pulse width.
  float total_injection_duration_ms_per_min = injections_per_minute * injectionLength_ms;

  // Convert the total open time from milliseconds per minute to minutes per minute.
  // This gives the fraction of each minute that the injector is open.
  float total_injection_duration_min_per_min = total_injection_duration_ms_per_min / 60000.0f;

  // Calculate fuel consumption in Liters per Minute by multiplying the duty cycle (as a fraction of a minute)
  // by the injector's flow rate.
  float fuel_consumption_L_per_min = total_injection_duration_min_per_min * INJECTOR_FLOW_RATE_L_PER_MIN;

  // Convert Liters per Minute to Liters per Hour.
  float fuel_consumption_L_per_hour = fuel_consumption_L_per_min * 60.0f;

  return fuel_consumption_L_per_hour;
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
  if (vehicleSpeed_kmh <= 0) {
    // Cannot calculate economy if the vehicle is not moving.
    // Consumption per distance is infinite. Return 0 for practical purposes.
    return 0;
  }

  // First, calculate consumption in Liters per Hour.
  float consumption_L_h = calculateFuelConsumption_LitersPerHour(injectionLength_ms, rpm);

  // Now, calculate economy based on speed.
  // Formula: (L/h) / (km/h) * 100 = L/100km
  float economy_L_100km = (consumption_L_h / vehicleSpeed_kmh) * 100.0f;

  return economy_L_100km;
}

/**
 * @brief Accumulates fuel consumption using the Kahan summation algorithm to maintain precision.
 *
 * @param consumption_L_h Instantaneous fuel consumption in Liters per Hour.
 * @param deltaTime_s Time elapsed since last update in seconds.
 */
void accumulateFuel(float consumption_L_h, float deltaTime_s) {
  if (deltaTime_s <= 0) return;

  // Calculate the small increment of fuel consumed during this interval
  float fuelIncrement = (consumption_L_h / 3600.0f) * deltaTime_s;

  // Kahan summation logic:
  // y is the increment plus the compensation from previous steps
  float y = fuelIncrement - kahanCompensation;
  // t is the intended sum
  float t = totalFuelConsumed_L + y;
  // (t - totalFuelConsumed_L) is the actual increment added, which might differ from y due to precision
  // Subtracting y from that difference gives the negative of the lost precision
  kahanCompensation = (t - totalFuelConsumed_L) - y;
  // Update the running total
  totalFuelConsumed_L = t;
}
