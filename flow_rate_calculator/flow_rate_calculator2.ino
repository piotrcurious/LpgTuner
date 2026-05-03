// LPG Flow Rate Calculator
// Implements a linearization and predictor function for LPG gas computers.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

class LPGFlowCalculator {
private:
    // --- Constants ---
    static constexpr float GAS_CONSTANT_K = 0.008314f; // Universal gas constant in kJ/(mol*K)
    static constexpr float MOLAR_MASS_LPG = 0.044f;    // Molar mass of LPG in kg/mol
    static constexpr float P_ATM = 1.01325f;           // Atmospheric pressure in bar
    static constexpr float BAR_TO_PA = 100000.0f;      // Conversion factor

    // --- Configuration ---
    float nozzle_diameter_mm;
    float injector_pulse_width_ms;
    float injections_per_minute;
    float nozzle_area_m2;

    // --- State Variables ---
    float gas_temperature_k = 300.0f;
    float gas_pressure_bar = 1.0f;      // P1
    float manifold_pressure_bar = 1.0f; // P2
    
    float gas_density = 0.0f;
    float flow_rate_kg_s = 0.0f;
    float delta_p_bar = 0.0f;
    float current_vacuum_kpa = 0.0f;
    float current_correction = 1.0f;

    // --- Tables & History ---
    static constexpr int N = 10;
    static constexpr float vacuum_table[N] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
    static constexpr float correction_table[N] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    // Circular buffer for history to avoid O(N) array shifting
    struct HistoryLog {
        float vacuum;
        float pressure;
        float correction;
    };
    HistoryLog history[N];
    int history_idx = 0;

    // --- Private Methods ---
    float interpolate(float x, float x1, float x2, float y1, float y2) {
        if (x2 == x1) return y1; // Prevent division by zero
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    void calculateGasDensity() {
        if (gas_temperature_k > 0.0f && gas_pressure_bar > 0.0f) {
            gas_density = (MOLAR_MASS_LPG * gas_pressure_bar) / (GAS_CONSTANT_K * gas_temperature_k * P_ATM);
        } else {
            gas_density = 0.0f;
        }
    }

    void calculateFlowRate() {
        delta_p_bar = gas_pressure_bar - manifold_pressure_bar;
        
        if (delta_p_bar > 0.0f && gas_density > 0.0f) {
            // A = (pi/4) * d^2
            float instantaneous_flow = nozzle_area_m2 * sqrt(2.0f * delta_p_bar * BAR_TO_PA / gas_density);
            
            // Apply duty cycle and correction
            float duty_cycle = (injector_pulse_width_ms / 1000.0f) * (injections_per_minute / 60.0f);
            flow_rate_kg_s = instantaneous_flow * duty_cycle * current_correction;
        } else {
            flow_rate_kg_s = 0.0f;
        }
    }

public:
    LPGFlowCalculator(float nozzle_dia_mm) : nozzle_diameter_mm(nozzle_dia_mm) {
        // Pre-calculate the nozzle area to save CPU cycles in the loop
        float d_m = nozzle_diameter_mm / 1000.0f;
        nozzle_area_m2 = 0.25f * PI * (d_m * d_m); 
        
        // Initialize history buffer
        for (int i = 0; i < N; i++) {
            history[i] = {0.0f, 1.0f, 1.0f};
        }
    }

    void updateSensors(float temp_k, float pressure_bar, float map_bar, float pulse_ms, float inj_per_min) {
        gas_temperature_k = temp_k;
        gas_pressure_bar = pressure_bar;
        manifold_pressure_bar = map_bar;
        injector_pulse_width_ms = pulse_ms;
        injections_per_minute = inj_per_min;
        
        current_vacuum_kpa = (1.0f - manifold_pressure_bar) * 100.0f;
    }

    void updateCorrections() {
        // Clamp vacuum to table boundaries to prevent out-of-bounds reading
        float clamped_vacuum = current_vacuum_kpa;
        if (clamped_vacuum < vacuum_table[0]) clamped_vacuum = vacuum_table[0];
        if (clamped_vacuum > vacuum_table[N - 1]) clamped_vacuum = vacuum_table[N - 1];

        // Find interpolation indices
        int idx = 0;
        for (int i = 0; i < N - 1; i++) {
            if (clamped_vacuum >= vacuum_table[i] && clamped_vacuum <= vacuum_table[i+1]) {
                idx = i;
                break;
            }
        }
        
        current_correction = interpolate(clamped_vacuum, vacuum_table[idx], vacuum_table[idx+1], 
                                         correction_table[idx], correction_table[idx+1]);

        // Save to circular buffer (avoids heavy array shifting)
        history[history_idx] = {current_vacuum_kpa, gas_pressure_bar, current_correction};
        history_idx = (history_idx + 1) % N; // Wrap around
    }

    void process() {
        updateCorrections();
        calculateGasDensity();
        calculateFlowRate();
    }

    // Getters
    float getDensity() const { return gas_density; }
    float getFlowRateGramsPerSec() const { return flow_rate_kg_s * 1000.0f; }
};

// --- Main Application ---

// Instantiate calculator with a 2.0mm nozzle
LPGFlowCalculator lpgCalc(2.0f);

void setup() {
    Serial.begin(115200);
}

void loop() {
    // 1. Read / Mock sensors
    float mock_temp_k = 313.15f; // 40C
    float mock_gas_pressure = 1.0f;
    float mock_map = 0.4f;
    float mock_pulse_width = 15.0f; 
    float mock_rpm_injections = 3000.0f; // e.g., 3000 RPM 

    // 2. Update and Process
    lpgCalc.updateSensors(mock_temp_k, mock_gas_pressure, mock_map, mock_pulse_width, mock_rpm_injections);
    lpgCalc.process();

    // 3. Print Results (Non-blocking)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        Serial.print("Density: "); 
        Serial.print(lpgCalc.getDensity(), 4);
        Serial.print(" kg/m3 | Flow: "); 
        Serial.print(lpgCalc.getFlowRateGramsPerSec(), 4); 
        Serial.println(" g/s");
    }

    // Keep loop tight, delay only slightly to prevent CPU hogging if necessary
    delay(10); 
}
