// Pin definitions
#define INJECTION_PULSE_PIN  34  // Analog input from the injection computer
#define LAMBDA1_PIN          35  // Analog input from lambda sensor 1
#define LAMBDA2_PIN          36  // Analog input from lambda sensor 2
#define DAC1_PIN             25  // DAC output for subgroup 1
#define DAC2_PIN             26  // DAC output for subgroup 2

// Digital output pins for narrowband lambda probe emulation
#define LAMBDA1_CONDITION_PIN  27  // Direction of drift (lean/rich)
#define LAMBDA1_ENABLE_PIN     14  // Enable signal for drift indication
#define LAMBDA2_CONDITION_PIN  12  // Limit exceeded condition (lean/rich)
#define LAMBDA2_ENABLE_PIN     13  // Enable signal for limit exceeded

// Desired target voltages for lean and rich conditions
#define LEAN_TARGET          0.45  // V for slightly lean condition
#define RICH_TARGET          0.65  // V for slightly rich condition

// DAC output constraints
#define DAC_MIN              0
#define DAC_MAX              255
#define DAC_MID              128

// Proportional gain for the controller
#define KP                   2.0

// Threshold for enabling the lambda probe emulation output
#define ERROR_THRESHOLD      0.05  // Voltage difference threshold for asymmetry detection
#define ASYMMETRY_THRESHOLD  10    // Threshold difference between offsets to trigger drift condition

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Setup analog inputs and DAC outputs
  analogReadResolution(12);  // ESP32 default ADC resolution is 12-bit (0-4095)
  analogSetAttenuation(ADC_11db);  // Full-scale voltage range (0-3.3V)

  // Setup digital pins for lambda probe emulation
  pinMode(LAMBDA1_CONDITION_PIN, INPUT);
  pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  pinMode(LAMBDA2_CONDITION_PIN, INPUT);
  pinMode(LAMBDA2_ENABLE_PIN, INPUT);
}

void loop() {
  // Read the current injection pulse length from the injection computer
  int injectionPulseRaw = analogRead(INJECTION_PULSE_PIN);
  float injectionPulseVoltage = injectionPulseRaw * (3.3 / 4095.0);

  // Read lambda sensors
  int lambda1Raw = analogRead(LAMBDA1_PIN);
  int lambda2Raw = analogRead(LAMBDA2_PIN);
  float lambda1Voltage = lambda1Raw * (3.3 / 4095.0);
  float lambda2Voltage = lambda2Raw * (3.3 / 4095.0);

  // Calculate the error for each subgroup
  float error1 = LEAN_TARGET - lambda1Voltage;  // Subgroup 1 should be lean
  float error2 = RICH_TARGET - lambda2Voltage;  // Subgroup 2 should be rich

  // Calculate the offset
  int offset1 = DAC_MID + (int)(KP * error1 * 128.0);
  int offset2 = DAC_MID + (int)(KP * error2 * 128.0);

  // Constrain the offset values to DAC limits
  offset1 = constrain(offset1, DAC_MIN, DAC_MAX);
  offset2 = constrain(offset2, DAC_MIN, DAC_MAX);

  // Output the offset values via DACs
  dacWrite(DAC1_PIN, offset1);
  dacWrite(DAC2_PIN, offset2);

  // Handle lambda probe emulation for asymmetry drift (Lambda1)
  if (abs(offset1 - offset2) > ASYMMETRY_THRESHOLD) {
    // Set the condition pin based on the direction of drift (which offset is higher)
    pinMode(LAMBDA1_CONDITION_PIN, OUTPUT);
    if (offset1 > offset2) {
      digitalWrite(LAMBDA1_CONDITION_PIN, HIGH);  // Drift towards rich
    } else {
      digitalWrite(LAMBDA1_CONDITION_PIN, LOW);   // Drift towards lean
    }
    // Enable the condition output
    pinMode(LAMBDA1_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA1_ENABLE_PIN, HIGH);
  } else {
    // Disable the output by setting pins as input
    pinMode(LAMBDA1_CONDITION_PIN, INPUT);
    pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  }

  // Handle lambda probe emulation for limit exceeded (Lambda2)
  bool limitExceeded = (offset1 == DAC_MIN || offset1 == DAC_MAX || offset2 == DAC_MIN || offset2 == DAC_MAX);
  if (limitExceeded) {
    // Set the condition pin based on which limit was exceeded
    pinMode(LAMBDA2_CONDITION_PIN, OUTPUT);
    if (offset1 == DAC_MAX || offset2 == DAC_MAX) {
      digitalWrite(LAMBDA2_CONDITION_PIN, HIGH);  // Rich limit exceeded
    } else {
      digitalWrite(LAMBDA2_CONDITION_PIN, LOW);   // Lean limit exceeded
    }
    // Enable the condition output
    pinMode(LAMBDA2_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA2_ENABLE_PIN, HIGH);
  } else {
    // Disable the output by setting pins as input
    pinMode(LAMBDA2_CONDITION_PIN, INPUT);
    pinMode(LAMBDA2_ENABLE_PIN, INPUT);
  }

  // Debugging output
  Serial.print("Lambda1: "); Serial.print(lambda1Voltage); Serial.print("V, Offset1: "); Serial.println(offset1);
  Serial.print("Lambda2: "); Serial.print(lambda2Voltage); Serial.print("V, Offset2: "); Serial.println(offset2);

  // Delay to allow time for the engine's conditions to stabilize
  delay(100);
}
