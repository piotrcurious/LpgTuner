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

// Define the output pins
const int narrowbandOutPin = 25;       // DAC1 (GPIO25)
const int widebandOutPin = 26;         // DAC2 (GPIO26)

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
const float fuelDensityAt20C = 0.74;   // Fuel density at 20°C in kg/L (assumed for gasoline)

// Fuel mass flow lookup table (16 points)
float pulseLengthTable[16] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5};  // ms
float fuelMassTable[16] = {0.0, 0.01, 0.03, 0.07, 0.12, 0.18, 0.26, 0.35, 0.45, 0.56, 0.68, 0.81, 0.95, 1.10, 1.26, 1.43};  // kg

// Variables for input values
float injPulseLength;       // Fuel injection pulse length (ms)
float mapSensorValue;       // Manifold absolute pressure (V)
float combustionChamberSize; // Volume of combustion chamber (L)
float airTemperature;       // Inlet air temperature (C)
float humidity;             // Relative humidity (%)
float altitude = 200.0;     // Altitude in meters (can be made dynamic if an altitude sensor is added)
float barometricPressure = 101325.0; // Barometric pressure in Pascals at sea level

// Variables for the outputs
float narrowbandLambdaOutput;
float widebandAFROutput;

// Combustion efficiency factor
float combustionEfficiency = 0.95;

// Simulated time delay and filter for lambda sensor response
float lambdaSensorDelay = 0.1;  // seconds (can be tuned)
float lambdaPrevious = 1.0;
float lambdaFilterCoeff = 0.9;  // smoothing coefficient (0-1, closer to 1 means more smoothing)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the pins
  pinMode(ignitionPin, INPUT);
  
  // Start the DAC outputs (on GPIO25 and GPIO26)
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  
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
      // Perform more accurate combustion calculations
      float airMassFlow = calculateAirMassFlow(mapSensorValue, airTemperature, humidity, combustionChamberSize);
      float fuelMassFlow = interpolateFuelMassFlow(injPulseLength, airTemperature);

      // Calculate the air-fuel ratio (AFR)
      float AFR = airMassFlow / (fuelMassFlow * combustionEfficiency);

      // Calculate the lambda value (lambda = AFR / Stoichiometric AFR)
      float lambda = AFR / stoichiometricAFR;

      // Simulate the response of the narrowband lambda sensor
      narrowbandLambdaOutput = simulateLambdaSensorResponse(lambda);
      
      // Output the lambda value for narrowband simulation using DAC1
      int dacValueNarrowband = (int)(narrowbandLambdaOutput * 255.0 / 3.3); // Convert to 8-bit DAC value
      dac_output_voltage(DAC_CHANNEL_1, dacValueNarrowband);

      // Simulate wideband AFR output directly related to AFR value
      widebandAFROutput = AFR / 20.0;  // Normalize AFR to a 0-5V range (assuming 0-20 AFR range)
      
      // Output the AFR value for wideband simulation using DAC2
      int dacValueWideband = (int)(widebandAFROutput * 255.0 / 5.0); // Convert to 8-bit DAC value
      dac_output_voltage(DAC_CHANNEL_2, dacValueWideband);

      // Debugging output
      Serial.print("Inj Pulse: "); Serial.print(injPulseLength);
      Serial.print(" MAP: "); Serial.print(mapSensorValue);
      Serial.print(" Chamber Size: "); Serial.print(combustionChamberSize);
      Serial.print(" Air Temp: "); Serial.print(airTemperature);
      Serial.print(" Humidity: "); Serial.print(humidity);
      Serial.print(" AFR: "); Serial.print(AFR);
      Serial.print(" Lambda: "); Serial.println(lambda);
      Serial.print(" Wideband AFR Output: "); Serial.println(widebandAFROutput);
    }
  } else {
    Serial.println("Failed to read from SHT31 sensor");
  }
  
  // Small delay to prevent overloading the serial output
  delay(10);
}

// Function to simulate the response of a real lambda sensor
float simulateLambdaSensorResponse(float lambda) {
  // Apply a simple time delay and filtering effect
  float delayedLambda = lambdaPrevious + (lambda - lambdaPrevious) * lambdaFilterCoeff;
  lambdaPrevious = delayedLambda;
  return delayedLambda;
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
float calculateAirMassFlow(float mapPressure, float airTempCelsius, float humidity, float chamberSizeL) {
    // Convert air temperature to Kelvin
    float airTempKelvin = airTempCelsius + 273.15;

    // Calculate the partial pressure of water vapor using the Magnus-Tetens approximation
    float saturationPressure = 610.78 * exp(17.27 * airTempCelsius / (airTempCelsius + 237.3));
    float actualVaporPressure = (humidity / 100.0) * saturationPressure;

    // Calculate the partial pressure of dry air
    float dryAirPressure = mapPressure - actualVaporPressure;

    // Calculate air density using the ideal gas law: ρ = P / (R * T)
    // Where P is the pressure, R is the specific gas constant, and T is the temperature in Kelvin
    float airDensity = (dryAirPressure / (Rd * airTempKelvin)) + (actualVaporPressure / (Rv * airTempKelvin)); // in kg/m^3

    // Convert combustion chamber size from liters to cubic meters
    float chamberSizeM3 = chamberSizeL / 1000.0;

    // Calculate air mass (mass = density * volume)
    float airMass = airDensity * chamberSizeM3; // in kg

    return airMass;
}

// Function to interpolate the fuel mass flow using the 16-point lookup table
float interpolateFuelMassFlow(float pulseLength, float airTempCelsius) {
    // Perform linear interpolation based on pulse length
    for (int i = 0; i < 15; i++) {
        if (pulseLength >= pulseLengthTable[i] && pulseLength <= pulseLengthTable[i + 1]) {
            float proportion = (pulseLength - pulseLengthTable[i]) / (pulseLengthTable[i + 1] - pulseLengthTable[i]);
            float interpolatedFuelMass = fuelMassTable[i] + proportion * (fuelMassTable[i + 1] - fuelMassTable[i]);
            return interpolatedFuelMass;
        }
    }
    // If pulse length is outside the defined range, return the closest value
    if (pulseLength < pulseLengthTable[0]) return fuelMassTable[0];
    if (pulseLength > pulseLengthTable[15]) return fuelMassTable[15];

    return 0.0; // Fallback
}
