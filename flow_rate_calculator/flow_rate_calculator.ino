// LPG Flow Rate Calculator
// This project implements a linearization and predictor function for LPG gas computers.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

// Constants
const float GAS_CONSTANT_K = 0.008314; // Universal gas constant in kJ/(mol*K)
const float MOLAR_MASS_LPG = 0.044; // Molar mass of LPG in kg/mol
const float P_ATM = 1.01325; // Atmospheric pressure in bar

// Configuration
float nozzle_diameter_mm = 2.0;
float injector_pulse_width_ms = 0;
float injections_per_minute = 0;
float gas_temperature_k = 300;

// Calculated variables
float gas_volume = 0;
float gas_density = 0;
float flow_rate_kg_s = 0;
float gas_pressure_bar = 1.0; // P1
float manifold_pressure_bar = 1.0; // P2
float delta_p_bar = 0;

// Dynamic corrections
const int N = 10;
float vacuum_table[N] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
float correction_table[N] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

float vacuum_array[N];
float pressure_array[N];
float correction_array[N];
float current_vacuum_kpa = 0;
float current_correction = 1.0;

void calculate_gas_density() {
    if (gas_temperature_k > 0 && gas_pressure_bar > 0) {
        gas_density = (MOLAR_MASS_LPG * gas_pressure_bar) / (GAS_CONSTANT_K * gas_temperature_k * P_ATM);
    }
}

void calculate_flow_rate() {
    float A = 0.25 * PI * pow(nozzle_diameter_mm / 1000.0, 2);
    delta_p_bar = gas_pressure_bar - manifold_pressure_bar;
    if (delta_p_bar > 0 && gas_density > 0) {
        float instantaneous_flow = A * sqrt(2 * delta_p_bar * 100000.0 / gas_density);
        flow_rate_kg_s = instantaneous_flow * (injector_pulse_width_ms / 1000.0) * (injections_per_minute / 60.0);
    } else {
        flow_rate_kg_s = 0;
    }
}

float interpolate(float x, float x1, float x2, float y1, float y2) {
    if (x2 == x1) return y1;
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

void update_corrections() {
    int idx = 0;
    for (int i = 0; i < N - 1; i++) {
        if (current_vacuum_kpa >= vacuum_table[i] && current_vacuum_kpa <= vacuum_table[i+1]) {
            idx = i;
            break;
        }
    }
    current_correction = interpolate(current_vacuum_kpa, vacuum_table[idx], vacuum_table[idx+1], correction_table[idx], correction_table[idx+1]);

    for (int i = N - 1; i > 0; i--) {
        vacuum_array[i] = vacuum_array[i - 1];
        pressure_array[i] = pressure_array[i - 1];
        correction_array[i] = correction_array[i - 1];
    }
    vacuum_array[0] = current_vacuum_kpa;
    pressure_array[0] = gas_pressure_bar;
    correction_array[0] = current_correction;
}

void setup() {
    Serial.begin(115200);
    for(int i=0; i<N; i++) {
        vacuum_array[i] = 0;
        pressure_array[i] = 1.0;
        correction_array[i] = 1.0;
    }
}

void loop() {
    // Mock sensor readings for now
    gas_temperature_k = 313.15; // 40C
    manifold_pressure_bar = 0.4;
    current_vacuum_kpa = (1.0 - manifold_pressure_bar) * 100.0;

    update_corrections();
    calculate_gas_density();
    calculate_flow_rate();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        Serial.print("Density: "); Serial.print(gas_density);
        Serial.print(" | Flow: "); Serial.print(flow_rate_kg_s * 1000.0); Serial.println(" g/s");
        lastPrint = millis();
    }
    delay(10);
}
