// LPG Flow Rate Estimator (Hagen-Poiseuille model)
// Estimates LPG flow rate based on injector pulse width and nozzle physics.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif
#include <math.h>

// --- Physical Constants (Precomputed at compile time) ---
constexpr float NOZZLE_DIAMETER_MM = 2.0;
constexpr float NOZZLE_RADIUS_M = (NOZZLE_DIAMETER_MM / 2.0) / 1000.0;
constexpr float GAS_CONSTANT_R = 8.314;     // J/(mol*K)
constexpr float MOLAR_MASS_LPG_KG = 0.0441; // kg/mol (44.1 g/mol)
constexpr float SPECIFIC_GAS_CONSTANT = GAS_CONSTANT_R / MOLAR_MASS_LPG_KG;
constexpr float BAR_TO_PASCAL = 100000.0;

// Precomputed Hagen-Poiseuille static multiplier: (pi * r^4) / (8 * R_specific)
constexpr float FLOW_CONST = (PI * pow(NOZZLE_RADIUS_M, 4)) / (8.0 * SPECIFIC_GAS_CONSTANT);

// --- Lookup Tables (Stored in Flash to save SRAM) ---
constexpr int TABLE_SIZE = 10;

const float pressure_table[TABLE_SIZE] = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9};
const float volume_table[TABLE_SIZE]   = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9};

const float vacuum_table[TABLE_SIZE]        = {-0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0};
const float pressure_correction[TABLE_SIZE] = {1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1};

const float flow_rate_correction[TABLE_SIZE] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// --- Helper Functions ---

/**
 * 1D Linear Interpolation for lookup tables. 
 * Provides smooth transitions between data points instead of jumping to nearest.
 */
float interpolate1D(float x, const float* x_axis, const float* y_axis, int size) {
    // Boundary checks (clamp to edges)
    if (x <= x_axis[0]) return y_axis[0];
    if (x >= x_axis[size - 1]) return y_axis[size - 1];

    // Find the interval and interpolate
    for (int i = 0; i < size - 1; i++) {
        if (x >= x_axis[i] && x <= x_axis[i + 1]) {
            float range = x_axis[i + 1] - x_axis[i];
            if (range == 0) return y_axis[i]; // Prevent division by zero
            
            float weight = (x - x_axis[i]) / range;
            return y_axis[i] + weight * (y_axis[i + 1] - y_axis[i]);
        }
    }
    return y_axis[size - 1]; // Fallback
}

float predict_pressure(float map_sensor_bar) {
    float vacuum = map_sensor_bar - 1.0;
    float correction = interpolate1D(vacuum, vacuum_table, pressure_correction, TABLE_SIZE);
    return map_sensor_bar + abs(vacuum * correction);
}

float estimate_flow_rate(float pressure_bar, float temperature_k, float pulse_width_ms, float injections_per_min) {
    if (temperature_k <= 0) return 0.0; // Prevent divide by zero

    // 1. Find Volume and related correction
    float volume_m3 = interpolate1D(pressure_bar, pressure_table, volume_table, TABLE_SIZE);
    float flow_corr = interpolate1D(volume_m3, volume_table, flow_rate_correction, TABLE_SIZE);

    // 2. Calculate Duty Cycle (Time open per second)
    // (pw in ms / 1000) * (injections / 60 seconds)
    float duty_cycle = (pulse_width_ms / 1000.0) * (injections_per_min / 60.0);

    // 3. Simplified Hagen-Poiseuille based estimate
    float pressure_pa = pressure_bar * BAR_TO_PASCAL;
    float flow_est = duty_cycle * FLOW_CONST * (pressure_pa / temperature_k);

    return flow_est * flow_corr;
}

// --- Arduino Core ---

void setup() {
    Serial.begin(115200);
}

void loop() {
    // 1. Read Sensors (localized variables)
    float raw_pressure_bar = analogRead(A0) / 102.3;
    float temperature_k    = (analogRead(A1) / 2.0) + 200.0; // Mock mapping
    float pulse_width_ms   = analogRead(A2) / 50.0;
    float inj_per_minute   = analogRead(A3) * 5.0;
    float map_sensor_bar   = analogRead(A4) / 1023.0;

    // 2. Process Data
    float corrected_pressure = predict_pressure(map_sensor_bar);
    
    // Note: Using raw_pressure_bar here as per original logic for volume, 
    // but you may want to use corrected_pressure depending on your physical model.
    float volume_m3 = interpolate1D(raw_pressure_bar, pressure_table, volume_table, TABLE_SIZE);
    float flow_rate_m3_s = estimate_flow_rate(corrected_pressure, temperature_k, pulse_width_ms, inj_per_minute);

    // 3. Non-blocking Serial Output (500ms intervals)
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= 500) {
        previousMillis = currentMillis;
        
        Serial.print("Vol: "); 
        Serial.print(volume_m3);
        Serial.print(" m^3 | Flow: "); 
        Serial.print(flow_rate_m3_s, 6);
        Serial.println(" m^3/s");
    }
}
