// Define the pins for the injector and the DACs
#define INJECTOR_PIN 2 // The pin connected to the injector signal
#define DAC1_PIN 25 // The pin connected to the LSB DAC
#define DAC2_PIN 26 // The pin connected to the MSB DAC

// Define the constants for the DAC resolution and voltage divider
#define DAC_RESOLUTION 8 // The number of bits for the DAC output
#define VOLTAGE_DIVIDER_RATIO 0.5 // The ratio of the voltage divider on the MSB DAC output

// Define the variables for storing the injector pulse width and the DAC values
volatile unsigned long pulseStart = 0; // The start time of the injector pulse in microseconds
volatile unsigned long pulseWidth = 0; // The width of the injector pulse in microseconds
volatile uint16_t dacValue = 0; // The combined value of the two DACs
volatile uint8_t dac1Value = 0; // The value of the LSB DAC
volatile uint8_t dac2Value = 0; // The value of the MSB DAC

// Define the interrupt service routine for measuring the injector pulse width
void ICACHE_RAM_ATTR measurePulse() {
  if (digitalRead(INJECTOR_PIN) == HIGH) { // If the injector signal is high, start measuring
    pulseStart = micros(); // Record the start time of the pulse
  } else { // If the injector signal is low, stop measuring
    pulseWidth = micros() - pulseStart; // Calculate the width of the pulse
    dacValue = map(pulseWidth, 0, 20000, 0, (1 << (2 * DAC_RESOLUTION)) - 1); // Map the pulse width to the DAC range
    dac1Value = dacValue & ((1 << DAC_RESOLUTION) - 1); // Extract the LSB bits of the DAC value
    dac2Value = (dacValue >> DAC_RESOLUTION) & ((1 << DAC_RESOLUTION) - 1); // Extract the MSB bits of the DAC value
    dacWrite(DAC1_PIN, dac1Value); // Write the LSB bits to the LSB DAC
    dacWrite(DAC2_PIN, dac2Value); // Write the MSB bits to the MSB DAC
  }
}

void setup() {
  pinMode(INJECTOR_PIN, INPUT); // Set the injector pin as input
  pinMode(DAC1_PIN, OUTPUT); // Set the LSB DAC pin as output
  pinMode(DAC2_PIN, OUTPUT); // Set the MSB DAC pin as output
  attachInterrupt(digitalPinToInterrupt(INJECTOR_PIN), measurePulse, CHANGE); // Attach an interrupt to the injector pin on both rising and falling edges
}

void loop() {
  // Nothing to do in the main loop
}
