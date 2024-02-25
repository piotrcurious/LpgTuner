// Arduino code for attiny85 implementing variable frequency variable pulse width software signal generator
// A0 controls frequency, A1 controls pulse width
// Based on https://www.instructables.com/Arduino-Variable-Frequency-PWM-Signal-Generator/
// dreamed by Copilot

// Define the pins
#define FREQ_PIN A0 // Analog input pin for frequency
#define PULSE_PIN A1 // Analog input pin for pulse width
#define OUTPUT_PIN 0 // Digital output pin for signal

// Define the constants
#define MIN_FREQ 1 // Minimum frequency in Hz
#define MAX_FREQ 1000 // Maximum frequency in Hz
#define MIN_PULSE 0 // Minimum pulse width in microseconds
#define MAX_PULSE 1000 // Maximum pulse width in microseconds

// Define the variables
int freq = 0; // Frequency value from FREQ_PIN
int pulse = 0; // Pulse width value from PULSE_PIN
int period = 0; // Period of the signal in microseconds
int onTime = 0; // On time of the signal in microseconds
int offTime = 0; // Off time of the signal in microseconds

void setup() {
  // Set the output pin as output
  pinMode(OUTPUT_PIN, OUTPUT);
}

void loop() {
  // Read the analog values from the pins
  freq = analogRead(FREQ_PIN);
  pulse = analogRead(PULSE_PIN);

  // Map the analog values to the desired ranges
  freq = map(freq, 0, 1023, MIN_FREQ, MAX_FREQ);
  pulse = map(pulse, 0, 1023, MIN_PULSE, MAX_PULSE);

  // Calculate the period, on time and off time of the signal
  period = 1000000 / freq; // Period in microseconds
  onTime = pulse; // On time in microseconds
  offTime = period - onTime; // Off time in microseconds

  // Generate the signal
  digitalWrite(OUTPUT_PIN, HIGH); // Turn on the output pin
  delayMicroseconds(onTime); // Wait for the on time
  digitalWrite(OUTPUT_PIN, LOW); // Turn off the output pin
  delayMicroseconds(offTime); // Wait for the off time
}
