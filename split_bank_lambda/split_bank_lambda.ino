#include <Arduino.h>
#include "ControllerLogic.h"

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
  // Run the core control logic
  runControllerStep();

  // Diagnostics
  static unsigned long lastDiag = 0;
  if (millis() - lastDiag >= 500) {
    lastDiag = millis();
    float drift = currentOffset1 - currentOffset2;
    bool atLimit = ((int)currentOffset1 <= DAC_MIN || (int)currentOffset1 >= DAC_MAX ||
                    (int)currentOffset2 <= DAC_MIN || (int)currentOffset2 >= DAC_MAX);

    Serial.printf("L1: %.2fV, L2: %.2fV | Off1: %d, Off2: %d | Drift: %.1f | Lim: %s\n",
                  filteredLambda1, filteredLambda2, (int)currentOffset1, (int)currentOffset2,
                  drift, atLimit ? "YES" : "NO");
  }
}
