#include <Arduino.h>

/*
 * Split Bank Lambda Controller for ESP32
 *
 * This controller manages two subgroups of cylinders to maintain slightly different AFRs.
 * Subgroup 1: Targeted for slightly lean (LEAN_TARGET)
 * Subgroup 2: Targeted for slightly rich (RICH_TARGET)
 *
 * Uses Sliding Mode Control (SMC) for robustness against non-linear narrowband lambda response.
 *
 * Emulation outputs:
 * - Probe 1: Indicates offset asymmetry (drift direction).
 * - Probe 2: Indicates if any controller has hit its output limits (DAC saturation).
 */

// Pin definitions
#define LAMBDA1_PIN          35  // Analog input from lambda sensor 1
#define LAMBDA2_PIN          36  // Analog input from lambda sensor 2
#define DAC1_PIN             25  // DAC output for subgroup 1
#define DAC2_PIN             26  // DAC output for subgroup 2

// Digital output pins for narrowband lambda probe emulation
// Emulates a virtual lambda probe behavior based on system state
#define LAMBDA1_CONDITION_PIN  27  // Condition signal (HIGH/LOW) for Drift
#define LAMBDA1_ENABLE_PIN     14  // Enable signal for Drift
#define LAMBDA2_CONDITION_PIN  12  // Condition signal (HIGH/LOW) for Limits
#define LAMBDA2_ENABLE_PIN     13  // Enable signal for Limits

// Desired target voltages for lean and rich conditions
// Note: Narrowband sensors switch sharply at ~0.45V.
#define LEAN_TARGET          0.45f  // Targeted for slightly lean/stoic edge
#define RICH_TARGET          0.65f  // Targeted for slightly rich

// DAC output constraints
#define DAC_MIN              0
#define DAC_MAX              255
#define DAC_MID              128

// Sliding Mode Control Parameters
#define SMC_GAIN             0.5f   // Rate of offset change (units per update). Reduced for stability.
#define SMC_BOUNDARY         0.10f  // Boundary layer (Volts) for chattering reduction. Increased.

// Thresholds for Emulation
#define ASYMMETRY_THRESHOLD  15.0f  // Difference in offsets to trigger drift emulation
#define FILTER_ALPHA         0.10f  // Simple EMA filter for analog inputs. Slightly more filtering.

// Global variables
float currentOffset1 = DAC_MID;
float currentOffset2 = DAC_MID;
float filteredLambda1 = 0.5f;
float filteredLambda2 = 0.5f;

unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 10; // 10ms = 100Hz loop

// Saturation function for Sliding Mode Control
// Returns values between -1.0 and 1.0.
// Uses a linear boundary layer to prevent high-frequency chattering.
float saturation(float s, float boundary) {
  if (s > boundary) return 1.0f;
  if (s < -boundary) return -1.0f;
  return s / boundary;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Split Bank Lambda Controller Starting (SMC Mode)...");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // 0-3.3V range

  // Emulation pins start as INPUT (High-Z) to avoid interfering with ECU
  pinMode(LAMBDA1_CONDITION_PIN, INPUT);
  pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  pinMode(LAMBDA2_CONDITION_PIN, INPUT);
  pinMode(LAMBDA2_ENABLE_PIN, INPUT);
}

void loop() {
  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_INTERVAL) return;
  lastUpdate = (now / UPDATE_INTERVAL) * UPDATE_INTERVAL; // Keep exact timing

  // 1. Read and filter inputs
  // Convert 12-bit ADC to Voltage (assuming 3.3V reference)
  float rawL1 = analogRead(LAMBDA1_PIN) * (3.3f / 4095.0f);
  float rawL2 = analogRead(LAMBDA2_PIN) * (3.3f / 4095.0f);

  // EMA filter to smooth out noise for the controller
  filteredLambda1 = (rawL1 * FILTER_ALPHA) + (filteredLambda1 * (1.0f - FILTER_ALPHA));
  filteredLambda2 = (rawL2 * FILTER_ALPHA) + (filteredLambda2 * (1.0f - FILTER_ALPHA));

  // 2. Sliding Mode Control Logic
  // Error s = Target - Measured
  float s1 = LEAN_TARGET - filteredLambda1;
  float s2 = RICH_TARGET - filteredLambda2;

  // SMC Update Rule: offset_dot = K * sat(s/boundary)
  // If Measured < Target (s > 0), we are too lean -> Increase fuel (increase offset)
  currentOffset1 += SMC_GAIN * saturation(s1, SMC_BOUNDARY);
  currentOffset2 += SMC_GAIN * saturation(s2, SMC_BOUNDARY);

  // Apply DAC constraints
  currentOffset1 = constrain(currentOffset1, (float)DAC_MIN, (float)DAC_MAX);
  currentOffset2 = constrain(currentOffset2, (float)DAC_MIN, (float)DAC_MAX);

  int out1 = (int)currentOffset1;
  int out2 = (int)currentOffset2;

  // 3. Update DAC outputs
  dacWrite(DAC1_PIN, out1);
  dacWrite(DAC2_PIN, out2);

  // 4. Emulation Logic (Narrowband Lambda Emulation)

  // A. Drift Emulation (Probe 1)
  // Triggered if the two subgroups require significantly different fuel offsets.
  // This indicates a physical imbalance between banks.
  float drift = currentOffset1 - currentOffset2;
  if (abs(drift) > ASYMMETRY_THRESHOLD) {
    pinMode(LAMBDA1_CONDITION_PIN, OUTPUT);
    digitalWrite(LAMBDA1_CONDITION_PIN, drift > 0 ? HIGH : LOW); // High if Bank 1 is richer than Bank 2
    pinMode(LAMBDA1_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA1_ENABLE_PIN, HIGH);
  } else {
    // Within tolerance: let the ECU rely on its own logic
    pinMode(LAMBDA1_CONDITION_PIN, INPUT);
    pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  }

  // B. Limit Emulation (Probe 2)
  // Triggered if any subgroup has reached the maximum or minimum compensation.
  // This notifies the ECU that the external controller has "run out of authority".
  bool atLimit = (out1 <= DAC_MIN || out1 >= DAC_MAX || out2 <= DAC_MIN || out2 >= DAC_MAX);
  if (atLimit) {
    pinMode(LAMBDA2_CONDITION_PIN, OUTPUT);
    // If we hit MAX, we are at the Rich-compensation limit (mixture is still too lean).
    // If we hit MIN, we are at the Lean-compensation limit (mixture is still too rich).
    if (out1 >= DAC_MAX || out2 >= DAC_MAX) {
      digitalWrite(LAMBDA2_CONDITION_PIN, HIGH); // Signal "Rich Limit" (High)
    } else {
      digitalWrite(LAMBDA2_CONDITION_PIN, LOW);  // Signal "Lean Limit" (Low)
    }
    pinMode(LAMBDA2_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA2_ENABLE_PIN, HIGH);
  } else {
    pinMode(LAMBDA2_CONDITION_PIN, INPUT);
    pinMode(LAMBDA2_ENABLE_PIN, INPUT);
  }

  // 5. Diagnostics
  static int diagCount = 0;
  if (++diagCount >= 50) { // Every 500ms
    diagCount = 0;
    Serial.printf("L1: %.2fV, L2: %.2fV | Off1: %d, Off2: %d | Drift: %.1f | Lim: %s\n",
                  filteredLambda1, filteredLambda2, out1, out2, drift, atLimit ? "YES" : "NO");
  }
}
