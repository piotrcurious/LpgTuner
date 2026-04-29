#ifndef CONTROLLER_LOGIC_H
#define CONTROLLER_LOGIC_H

/*
 * Shared definitions and logic for the Split Bank Lambda Controller.
 * This file is shared between the Arduino sketch and the C++ simulation.
 */

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cmath>
#include <algorithm>
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

// Pin definitions
#define LAMBDA1_PIN          35  // Analog input from lambda sensor 1
#define LAMBDA2_PIN          36  // Analog input from lambda sensor 2
#define DAC1_PIN             25  // DAC output for subgroup 1
#define DAC2_PIN             26  // DAC output for subgroup 2

// Digital output pins for narrowband lambda probe emulation
#define LAMBDA1_CONDITION_PIN  27  // Condition signal (HIGH/LOW) for Drift
#define LAMBDA1_ENABLE_PIN     14  // Enable signal for Drift
#define LAMBDA2_CONDITION_PIN  12  // Condition signal (HIGH/LOW) for Limits
#define LAMBDA2_ENABLE_PIN     13  // Enable signal for Limits

// Desired target voltages for lean and rich conditions
#define LEAN_TARGET          0.45f
#define RICH_TARGET          0.65f

// DAC output constraints
#define DAC_MIN              0
#define DAC_MAX              255
#define DAC_MID              128

// Sliding Mode Control Parameters
#define SMC_GAIN             0.2f   // Rate of offset change. Reduced for idle stability.
#define SMC_BOUNDARY         0.20f  // Boundary layer (Volts) for chattering reduction. Increased.

// Thresholds for Emulation
#define ASYMMETRY_THRESHOLD  40.0f  // Difference in offsets to trigger drift emulation
#define FILTER_ALPHA         0.10f  // Simple EMA filter for analog inputs

// Global state variables
extern float currentOffset1;
extern float currentOffset2;
extern float filteredLambda1;
extern float filteredLambda2;
extern unsigned long lastUpdate;

const unsigned long UPDATE_INTERVAL = 10; // 10ms = 100Hz loop

// Saturation function for Sliding Mode Control
inline float saturation(float s, float boundary) {
  if (s > boundary) return 1.0f;
  if (s < -boundary) return -1.0f;
  return s / boundary;
}

// Logic update function (to be called in loop)
// Requires external implementation of analogRead, dacWrite, digitalWrite, pinMode, millis
void runControllerStep();

#endif
