#include <Arduino.h>

/* * ESP32 Pin Configuration 
 * Note: Use ADC1 pins (GPIO 32-39) if using WiFi, as ADC2 is used by WiFi.
 */
#define INJECTION_PULSE_PIN  34  // Analog Input: Original pulse length from ECU (0-3.3V)
#define LAMBDA1_PIN          35  // Analog Input: Subgroup 1 Narrowband Sensor
#define LAMBDA2_PIN          36  // Analog Input: Subgroup 2 Narrowband Sensor

/* DAC Outputs (Fixed pins on ESP32) */
#define DAC1_PIN             25  // DAC Output: Offset Subgroup 1
#define DAC2_PIN             26  // DAC Output: Offset Subgroup 2

/* * Digital Output pins for Narrowband Lambda Probe Emulation 
 * We use 2 pairs of pins to define state and active status.
 */
// Emulation 1: Indicates direction of Asymmetry Drift
#define EM1_SIGNAL_PIN       27  // Logic LOW = Lean drift, HIGH = Rich drift
#define EM1_ACTIVE_PIN       14  // Set to OUTPUT to enable signal, INPUT to high-Z

// Emulation 2: Indicates DAC Limit Exceeded (Saturation)
#define EM2_SIGNAL_PIN       12  // Logic LOW = Hit Lean limit (trying to add fuel), HIGH = Hit Rich limit
#define EM2_ACTIVE_PIN       13  // Set to OUTPUT to enable signal, INPUT to high-Z

/* --- Control Constants --- */

// Target voltages from narrowband sensors to achieve slight bias
// Narrowband stoichiometric is approx 0.45V - 0.5V
#define TARGET_LEAN_VOLTS    0.30  // Target for slightly lean subgroup
#define TARGET_RICH_VOLTS    0.70  // Target for slightly rich subgroup

// ADC/DAC constants
#define ESP_ADC_RES          4095.0
#define ESP_VREF             3.3
#define DAC_MID              128   // Baseline (0 offset) per requirements
#define DAC_MIN              0     // Max fuel reduction limit
#define DAC_MAX              255   // Max fuel increase limit

// Controller Tuning
#define KP                   5.0   // Proportional Gain

// Emulation Logic Thresholds
// If |Offset1 - Offset2| exceeds this, treat as asymmetrical drift
#define ASYMMETRY_THRESHOLD  25    // (roughly 10% of DAC range)

/* --- Global Variables --- */
float currentOffset1 = DAC_MID;
float currentOffset2 = DAC_MID;

void setup() {
  Serial.begin(115200);

  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // Allow input up to ~3.1V

  // Initialize emulation pins to High-Impedance (Disabled) state
  pinMode(EM1_ACTIVE_PIN, INPUT);
  pinMode(EM1_SIGNAL_PIN, INPUT);
  pinMode(EM2_ACTIVE_PIN, INPUT);
  pinMode(EM2_SIGNAL_PIN, INPUT);

  Serial.println("Dual Subgroup Injection Offset Controller Initialized");
}

void loop() {
  // 1. Read Inputs
  int injRaw = analogRead(INJECTION_PULSE_PIN);
  int lamsRaw1 = analogRead(LAMBDA1_PIN);
  int lamsRaw2 = analogRead(LAMBDA2_PIN);

  // Convert to Voltages
  float injVolts = (injRaw / ESP_ADC_RES) * ESP_VREF;
  float lamVolts1 = (lamsRaw1 / ESP_ADC_RES) * ESP_VREF;
  float lamVolts2 = (lamsRaw2 / ESP_ADC_RES) * ESP_VREF;

  // 2. Control Logic
  
  // Calculate errors based on specific target for each bank
  // Error = Target - Actual. 
  // If Actual is leaner than Target, Error is positive (need to add fuel).
  float error1 = TARGET_LEAN_VOLTS - lamVolts1; 
  float error2 = TARGET_RICH_VOLTS - lamVolts2;

  // P-Controller calculation scaled to DAC counts
  // A positive error increases the offset above 128 (adds fuel)
  currentOffset1 = DAC_MID + (error1 * KP * 100.0);
  currentOffset2 = DAC_MID + (error2 * KP * 100.0);

  // 3. Apply Constraints & Detect Saturation (for Emulation 2)
  bool limitExceeded = false;
  bool limitDirectionRich = false; // Is the ESP trying to correct a rich condition (cutting fuel)?

  // Check Subgroup 1 Limits
  if (currentOffset1 >= DAC_MAX) {
    currentOffset1 = DAC_MAX;
    limitExceeded = true;
    limitDirectionRich = false; // Stuck trying to add fuel -> engine is too lean
  } else if (currentOffset1 <= DAC_MIN) {
    currentOffset1 = DAC_MIN;
    limitExceeded = true;
    limitDirectionRich = true; // Stuck trying to cut fuel -> engine is too rich
  }

  // Check Subgroup 2 Limits
  if (currentOffset2 >= DAC_MAX) {
    currentOffset2 = DAC_MAX;
    limitExceeded = true;
    limitDirectionRich = false;
  } else if (currentOffset2 <= DAC_MIN) {
    currentOffset2 = DAC_MIN;
    limitExceeded = true;
    limitDirectionRich = true;
  }

  // 4. Output DAC offsets
  dacWrite(DAC1_PIN, (int)currentOffset1);
  dacWrite(DAC2_PIN, (int)currentOffset2);

  // 5. Lambda Emulation Logic

  // --- Emulation 1: Asymmetry Drift ---
  float offsetDiff = currentOffset1 - currentOffset2;

  if (abs(offsetDiff) > ASYMMETRY_THRESHOLD) {
    // Enable output
    pinMode(EM1_SIGNAL_PIN, OUTPUT);
    pinMode(EM1_ACTIVE_PIN, OUTPUT);
    digitalWrite(EM1_ACTIVE_PIN, HIGH); // Pull-up emulation circuit active

    // Determine direction of drift. 
    // If Offset1 > Offset2, Subgroup 1 is demanding more fuel than 2 relative to targets.
    if (offsetDiff > 0) {
      // Overall drift requires more fuel (Engine is leaning out, ESP is correcting Rich)
      digitalWrite(EM1_SIGNAL_PIN, LOW); // Emulate Lean
    } else {
      digitalWrite(EM1_SIGNAL_PIN, HIGH); // Emulate Rich
    }
  } else {
    // within symmetry, disable output (High-Z)
    pinMode(EM1_ACTIVE_PIN, INPUT);
    pinMode(EM1_SIGNAL_PIN, INPUT);
  }

  // --- Emulation 2: Limit Exceeded (Saturation) ---
  if (limitExceeded) {
    // Enable output
    pinMode(EM2_SIGNAL_PIN, OUTPUT);
    pinMode(EM2_ACTIVE_PIN, OUTPUT);
    digitalWrite(EM2_ACTIVE_PIN, HIGH);

    // If controller is stuck at MIN, engine is too rich. If stuck at MAX, engine is too lean.
    if (limitDirectionRich) {
      digitalWrite(EM2_SIGNAL_PIN, HIGH); // Emulate Rich
    } else {
      digitalWrite(EM2_SIGNAL_PIN, LOW);  // Emulate Lean
    }
  } else {
    // Within limits, disable output (High-Z)
    pinMode(EM2_ACTIVE_PIN, INPUT);
    pinMode(EM2_SIGNAL_PIN, INPUT);
  }

  // Debugging
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 200) {
    Serial.print("In:"); Serial.print(injVolts);
    Serial.print("V | L1:"); Serial.print(lamVolts1);
    Serial.print("V, Off1:"); Serial.print((int)currentOffset1);
    Serial.print(" | L2:"); Serial.print(lamVolts2);
    Serial.print("V, Off2:"); Serial.print((int)currentOffset2);
    if(limitExceeded) Serial.print(" [SAT]");
    if(abs(offsetDiff) > ASYMMETRY_THRESHOLD) Serial.print(" [ASYM]");
    Serial.println();
    lastLog = millis();
  }

  // Small delay for stability
  delay(10); 
}
