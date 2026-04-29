#include <Arduino.h>

// PIN CONFIGURATION
const int inputPulsePin = A0;      // Analog input from injection computer
const int lambdaSensor1Pin = A1;   // Lambda sensor 1 analog input
const int lambdaSensor2Pin = A2;   // Lambda sensor 2 analog input
const int dacOutput1Pin = 25;      // DAC 1 output (Subgroup 1 offset)
const int dacOutput2Pin = 26;      // DAC 2 output (Subgroup 2 offset)
const int emulationOutputPin = 27;  // Digital output for emulated lambda signal

// SYSTEM CONSTANTS
const int offsetBaseline = 128;   // 128 is 0 offset
const float controlStep = 0.5;      // Amount of offset adjustment per loop
const int dacMin = 0;              // Minimum DAC value
const int dacMax = 255;            // Maximum DAC value

// TARGET AIR-FUEL RATIO (AFR) VALUES
// Adjust these based on the characteristics of your engine and sensors.
// A common definition for narrowband sensors is that 0.4-0.6V is stoichiometric (λ = 1).
const int leanLambdaTarget = 400;   // Target reading for slightly lean condition (e.g., 0.4V)
const int richLambdaTarget = 600;   // Target reading for slightly rich condition (e.g., 0.6V)

// CONTROL LOOP TIMING
const unsigned long loopDelayMillis = 100;   // Delay between loops in milliseconds
unsigned long lastLoopTime = 0;

// STATE VARIABLES
float dac1Value = offsetBaseline;
float dac2Value = offsetBaseline;

void setup() {
  Serial.begin(115200);

  // Configure output pins
  pinMode(dacOutput1Pin, OUTPUT);
  pinMode(dacOutput2Pin, OUTPUT);
  pinMode(emulationOutputPin, OUTPUT);

  // Initialize emulation output pin to a default state (e.g., low)
  digitalWrite(emulationOutputPin, LOW);
}

void loop() {
  // Check if enough time has passed to run the control loop
  if (millis() - lastLoopTime < loopDelayMillis) {
    return;
  }
  lastLoopTime = millis();

  // READ INPUTS
  int inputPulseLength = analogRead(inputPulsePin);
  int lambda1Reading = analogRead(lambdaSensor1Pin);
  int lambda2Reading = analogRead(lambdaSensor2Pin);

  // CALCULATE DAC OFFSETS
  // Maintain slight lean reading in subgroup 1 (e.g., higher pulse length if rich)
  if (lambda1Reading > leanLambdaTarget) {
    dac1Value += controlStep;
  } else if (lambda1Reading < leanLambdaTarget) {
    dac1Value -= controlStep;
  }

  // Maintain slight rich reading in subgroup 2 (e.g., lower pulse length if lean)
  if (lambda2Reading > richLambdaTarget) {
    dac2Value -= controlStep;
  } else if (lambda2Reading < richLambdaTarget) {
    dac2Value += controlStep;
  }

  // CONSTRAINT CHECKING AND EMULATION
  bool constraintsExceeded = false;

  // Check if offsets are symmetrical around the baseline
  float averageDacValue = (dac1Value + dac2Value) / 2.0;
  if (abs(averageDacValue - offsetBaseline) > offsetBaseline * 0.1) {
    // Symmetrical offset check: Is the average offset significantly different from 128?
    // You might want to adjust this check based on your desired level of symmetry.
    constraintsExceeded = true;
  }

  // Check if DAC values are within the constraints
  if (dac1Value < dacMin || dac1Value > dacMax || dac2Value < dacMin || dac2Value > dacMax) {
    constraintsExceeded = true;
  }

  // HANDLE CONSTRAINTS AND EMULATION
  if (constraintsExceeded) {
    // Set DAC values to a safe default if constraints are exceeded (e.g., baseline)
    dac1Value = offsetBaseline;
    dac2Value = offsetBaseline;

    // EMULATE NARROWBAND LAMBDA SIGNAL (Example: Toggle emulationOutputPin based on time)
    // You'll likely need a more complex signal based on your specific requirements.
    if ((millis() / 500) % 2 == 0) {
      digitalWrite(emulationOutputPin, HIGH);
    } else {
      digitalWrite(emulationOutputPin, LOW);
    }
  } else {
    // If within constraints, ensure DAC values stay within limits
    dac1Value = max(dacMin, min(dacMax, (int)dac1Value));
    dac2Value = max(dacMin, min(dacMax, (int)dac2Value));
    
    // Turn off emulation signal when operating within constraints
    digitalWrite(emulationOutputPin, LOW);
  }

  // UPDATE DAC OUTPUTS
  analogWrite(dacOutput1Pin, dac1Value);
  analogWrite(dacOutput2Pin, dac2Value);

  // (Optional) PRINT MONITORING DATA TO SERIAL
  Serial.print("Input: "); Serial.print(inputPulseLength);
  Serial.print(" | L1: "); Serial.print(lambda1Reading);
  Serial.print(" | L2: "); Serial.print(lambda2Reading);
  Serial.print(" | DAC1: "); Serial.print(dac1Value);
  Serial.print(" | DAC2: "); Serial.print(dac2Value);
  if (constraintsExceeded) {
    Serial.print(" | EMULATION ON");
  } else {
    Serial.print(" | EMULATION OFF");
  }
  Serial.println();
}
