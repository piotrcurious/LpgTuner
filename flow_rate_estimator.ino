// Define constants and variables
const float nozzle_diameter = 0.5; // nozzle diameter in mm (fixed)
const float gas_constant = 8.314; // universal gas constant in J/mol*K
const float molar_mass = 44.1; // molar mass of LPG in g/mol
const int table_size = 10; // size of the correction tables and arrays
float pressure_table[table_size]; // table of pressure values for calibration
float volume_table[table_size]; // table of volume values for calibration
float vacuum_table[table_size]; // table of vacuum values for correction
float pressure_correction[table_size]; // table of pressure correction factors
float volume_correction[table_size]; // table of volume correction factors
float pressure; // gas pressure in bar
float temperature; // gas temperature in K
float volume; // gas volume in m^3
float flow_rate; // gas flow rate in m^3/s
float pulse_width; // gasoline injector pulse width in ms
float injections_per_minute; // gasoline injections per minute
float map_sensor; // manifold absolute pressure sensor reading in bar

// Define functions

// Self calibrating linearization function for LPG gas computer program
// This function estimates gas volume from gas pressure and temperature using the ideal gas law
// It also calibrates the volume estimate using a table of pressure and volume values obtained from a reference source
// The function returns the calibrated gas volume in m^3
float linearize_volume(float pressure, float temperature) {
  float volume_estimate = (pressure * 100000) * (1 / (gas_constant / molar_mass)) * (1 / temperature); // calculate volume estimate using ideal gas law
  float pressure_error = 0; // initialize pressure error variable
  float volume_error = 0; // initialize volume error variable
  float calibrated_volume = 0; // initialize calibrated volume variable
  
  // Loop through the pressure table to find the closest pressure value to the input pressure
  for (int i = 0; i < table_size; i++) {
    float current_error = abs(pressure - pressure_table[i]); // calculate the absolute error between the input pressure and the current table value
    if (i == 0 || current_error < pressure_error) { // if this is the first iteration or the current error is smaller than the previous error
      pressure_error = current_error; // update the pressure error variable with the current error value
      volume_error = abs(volume_estimate - volume_table[i]); // update the volume error variable with the absolute error between the volume estimate and the corresponding table value
      calibrated_volume = volume_table[i]; // update the calibrated volume variable with the corresponding table value
    }
  }
  
  // Return the calibrated volume value
  return calibrated_volume;
}

// Additional predictor function for gas pressure
// This function estimates gas pressure from manifold absolute pressure sensor reading and vacuum line connection to LPG reductor
// It also applies dynamic corrections using tables of vacuum and pressure correction factors based on non-linear and non-instant responses of vacuum and reductor
// The function returns the corrected gas pressure in bar
float predict_pressure(float map_sensor) {
  float vacuum_estimate = map_sensor - 1; // calculate vacuum estimate from map sensor reading (assuming atmospheric pressure is 1 bar)
  float vacuum_error = 0; // initialize vacuum error variable
  float pressure_correction_factor = 0; // initialize pressure correction factor variable
  
  // Loop through the vacuum table to find the closest vacuum value to the vacuum estimate
  for (int i = 0; i < table_size; i++) {
    float current_error = abs(vacuum_estimate - vacuum_table[i]); // calculate the absolute error between the vacuum estimate and the current table value
    if (i == 0 || current_error < vacuum_error) { // if this is the first iteration or the current error is smaller than the previous error
      vacuum_error = current_error; // update the vacuum error variable with the current error value
      pressure_correction_factor = pressure_correction[i]; // update the pressure correction factor variable with the corresponding table value
    }
  }
  
  float corrected_pressure = map_sensor + (vacuum_estimate * pressure_correction_factor); // calculate corrected pressure by adding the map sensor reading and the product of the vacuum estimate and the pressure correction factor
  // Return the corrected pressure value
  return corrected_pressure;
}

// Function to estimate gas flow rate from gasoline injector pulse width, nozzle diameter and injections per minute
// This function assumes that the gas flow is laminar and uses the Hagen-Poiseuille equation to calculate the flow rate
// It also applies dynamic corrections using tables of volume and flow rate correction factors based on non-linear and non-instant responses of gas volume and flow rate
// The function returns the corrected gas flow rate in m^3/s
float estimate_flow_rate(float pulse_width, float injections_per_minute) {
  float volume_estimate = linearize_volume(pressure, temperature); // calculate volume estimate using the linearize_volume function
  float volume_error = 0; // initialize volume error variable
  float flow_rate_correction_factor = 0; // initialize flow rate correction factor variable
  
  // Loop through the volume table to find the closest volume value to the volume estimate
  for (int i = 0; i < table_size; i++) {
    float current_error = abs(volume_estimate - volume_table[i]); // calculate the absolute error between the volume estimate and the current table value
    if (i == 0 || current_error < volume_error) { // if this is the first iteration or the current error is smaller than the previous error
      volume_error = current_error; // update the volume error variable with the current error value
      flow_rate_correction_factor = flow_rate_correction[i]; // update the flow rate correction factor variable with the corresponding table value
    }
  }
  
  float flow_rate_estimate = (pulse_width / 1000) * (injections_per_minute / 60) * (M_PI * pow(nozzle_diameter / 2000, 4) * pressure * 100000) / (8 * gas_constant / molar_mass * temperature); // calculate flow rate estimate using Hagen-Poiseuille equation
  float corrected_flow_rate = flow_rate_estimate * flow_rate_correction_factor; // calculate corrected flow rate by multiplying the flow rate estimate and the flow rate correction factor
  // Return the corrected flow rate value
  return corrected_flow_rate;
}

// Main loop
void loop() {
  // Read sensor values and store them in variables
  pressure = analogRead(A0); // read gas pressure from analog pin A0 and convert it to bar (assuming a linear relationship between voltage and pressure)
  temperature = analogRead(A1); // read gas temperature from analog pin A1 and convert it to K (assuming a linear relationship between voltage and temperature)
  pulse_width = analogRead(A2); // read gasoline injector pulse width from analog pin A2 and convert it to ms (assuming a linear relationship between voltage and pulse width)
  injections_per_minute = analogRead(A3); // read gasoline injections per minute from analog pin A3 and convert it to number of injections (assuming a linear relationship between voltage and injections per minute)
  map_sensor = analogRead(A4); // read manifold absolute pressure sensor reading from analog pin A4 and convert it to bar (assuming a linear relationship between voltage and pressure)
  
  // Call functions to estimate gas volume, gas pressure and gas flow rate
  volume = linearize_volume(pressure, temperature); // call linearize_volume function and store the result in volume variable
  pressure = predict_pressure(map_sensor); // call predict_pressure function and store the result in pressure variable
  flow_rate = estimate_flow_rate(pulse_width, injections_per_minute); // call estimate_flow_rate function and store the result in flow_rate variable
  
  // Print results to serial monitor for debugging purposes
  Serial.print("Gas volume: ");
  Serial.print(volume);
  Serial.println(" m^3");
  
  Serial.print("Gas pressure: ");
  Serial.print(pressure);
  Serial.println(" bar");
  
  Serial.print("Gas flow rate: ");
  Serial.print(flow_rate);
  Serial.println(" m^3/s");
  
}
