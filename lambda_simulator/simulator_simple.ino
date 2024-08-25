#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <driver/adc.h>    // ESP32 ADC driver
#include <driver/dac.h>    // ESP32 DAC driver
#include <math.h>          // For mathematical functions like log

// Define the input pins
const int injPulsePin = 32;            // ADC1_CH4
const int mapSensorPin = 33;           // ADC1_CH5
const int combustionChamberPin = 34;   // ADC1_CH6
const int airTempPin = 35;             // ADC1_CH7 (Thermistor input)
const int ignitionPin = 26;            // Digital input for ignition signal

// Define the output pin
const int lambdaOutPin = 25;           // DAC1 (GPIO25)

// Initialize SHT31 sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Lambda look-up table (Bosch narrowband lambda probe)
// This is a basic example with fictitious values. You need to use the actual data from the lambda probe.
float lambdaTable[10] = {0.1, 0.3, 0.5, 0.7, 0.9, 1.0, 1.1, 1.3, 1.5, 1.7};

// Constants
const float stoichiometricAFR = 14.7;  // Stoichiometric air-fuel ratio for gasoline
const float R = 287.05;                // Specific gas constant for dry air (J/kg*K)
const float Rd = 287.05;               // Specific gas constant for dry air (J/kg*K)
const float Rv = 461.5;                // Specific gas constant for water vapor (J/kg*K)

// Fuel mass flow lookup table (16 points)
float pulseLengthTable[16] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5};  // ms
float fuelMassTable[16] = {0.0, 0.01, 0.03, 0.07, 0.12, 0.18, 0.26, 0.35, 0.45, 0.56, 0.68, 0.81, 0.95, 1.10, 1.26, 1.43};  // kg

// Variables for input values
float injPulseLength;       // Fuel injection pulse length (ms)
float mapSensorValue;       // Manifold absolute pressure (V)
float combustionChamberSize; // Volume of combustion chamber (L)
float airTemperature;       // Inlet air temperature (C)
float humidity;             // Relative humidity (%)

// Variable for the lambda output
float lambdaOutput;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the pins
  pinMode(ignitionPin, INPUT);
  
  // Start the DAC output (on GPIO25)
  dac_output_enable(DAC_CHANNEL_1);
  
  // Initialize SHT31 sensor
  if (!sht31.begin(0x44)) {   // Set the address to 0x44
    Serial.println("Couldn't find SHT31 sensor!");
    while (1) delay(1);
  }
}

void loop() {
  // Read inputs
  injPulseLength = analogRead(injPulsePin) * (3.3 / 4095.0); // Convert ADC value to voltage
  mapSensorValue = analogRead(mapSensorPin) * (3.3 / 4095.0); // Convert ADC value to voltage
  combustionChamberSize = analogRead(combustionChamberPin) * (3.3 / 4095.0); // Convert ADC value to voltage
  
  // Read temperature and humidity from SHT31 sensor
  airTemperature = sht31.readTemperature();
  humidity = sht31.readHumidity();

  if (!isnan(airTemperature) && !isnan(humidity)) {
    // Wait for ignition signal
    if (digitalRead(ignitionPin) == HIGH) {
      // Perform a more realistic combustion calculation with humidity consideration
      float airMassFlow = calculateAirMassFlow(mapSensorValue, airTemperature, humidity, combustionChamberSize);
      float fuelMassFlow = interpolateFuelMassFlow(injPulseLength);

      // Calculate the air-fuel ratio (AFR)
      float AFR = airMassFlow / fuelMassFlow;

      // Calculate the lambda value (lambda = AFR / Stoichiometric AFR)
      float lambda = AFR / stoichiometricAFR;

      // Lookup the corresponding lambda value in the lambda table
      lambdaOutput = mapLambda(lambda);

      // Output the lambda value using DAC
      int dacValue = (int)(lambdaOutput * 255.0 / 3.3); // Convert to 8-bit DAC value
      dac_output_voltage(DAC_CHANNEL_1, dacValue);

      // Debugging output
      Serial.print("Inj Pulse: "); Serial.print(injPulseLength);
      Serial.print(" MAP: "); Serial.print(mapSensorValue);
      Serial.print(" Chamber Size: "); Serial.print(combustionChamberSize);
      Serial.print(" Air Temp: "); Serial.print(airTemperature);
      Serial.print(" Humidity: "); Serial.print(humidity);
      Serial.print(" AFR: "); Serial.print(AFR);
      Serial.print(" Lambda: "); Serial.println(lambda);
    }
  } else {
    Serial.println("Failed to read from SHT31 sensor");
  }
  
  // Small delay to prevent overloading the serial output
  delay(10);
}

// Function to map lambda value using the lookup table
float mapLambda(float lambda) {
  // This is a simplified linear interpolation based on lambda value
  int index = (int)(lambda * 10.0); // Scale and clamp to table size
  if (index < 0) index = 0;
  if (index > 9) index = 9;
  return lambdaTable[index];
}

// Function to calculate air mass flow based on MAP, temperature, humidity, and combustion chamber size
float calculateAirMassFlow(float mapVoltage, float airTempCelsius, float humidity, float chamberSizeL) {
  // Convert MAP voltage to pressure (Pa)
  float mapPressure = mapVoltage * 100000.0; // Assume linear relationship and 100 kPa max pressure

  // Convert air temperature to Kelvin
  float airTempKelvin = airTempCelsius + 273.15;

  // Calculate the partial pressure of water vapor
  float saturationPressure = 610.78 * exp(airTempKelvin / 273.15 * 17.27 * (airTempCelsius / (airTempCelsius + 237.3)));
  float actualVaporPressure = (humidity / 100.0) * saturationPressure;

  // Calculate the partial pressure of dry air
  float dryAirPressure = mapPressure - actualVaporPressure;

  // Calculate air density using the equation: ρ = (Pd / Rd * T) + (Pv / Rv * T)
  float airDensity = (dryAirPressure / (Rd * airTempKelvin)) + (actualVaporPressure / (Rv * airTempKelvin)); // in kg/m^3

  // Convert combustion chamber size from liters to cubic meters
  float chamberSizeM3 = chamberSizeL / 1000.0;

  // Calculate air mass (mass = density * volume)
  float airMass = airDensity * chamberSizeM3; // in kg

  return airMass;
}

// Function to interpolate fuel mass flow based on injection pulse length using a 16-point lookup table
float interpolateFuelMassFlow(float pulseLength) {
  // Ensure pulse length is within the range of the table
  if (pulseLength <= pulseLengthTable[0]) return fuelMassTable[0];
  if (pulseLength >= pulseLengthTable[15]) return fuelMassTable[15];

  // Find the two points in the table that the pulse length falls between
  for (int i = 0; i < 15; i++) {
    if (pulseLength >= pulseLengthTable[i] && pulseLength < pulseLengthTable[i + 1]) {
      // Interpolate between the two points
      float slope = (fuelMassTable[i + 1] - fuelMassTable[i]) / (pulseLengthTable[i + 1] - pulseLengthTable[i]);
      return fuelMassTable[i] + slope * (pulseLength - pulseLengthTable[i]);
    }
  }

  // Default return value (shouldn't be reached)
  return 0.0;
}
