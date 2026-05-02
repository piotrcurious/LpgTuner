// LPG Flow Rate Estimator (Hagen-Poiseuille model)
// Estimates LPG flow rate based on injector pulse width and nozzle physics.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

// Constants
const float nozzle_diameter_mm = 2.0;
const float gas_constant_R = 8.314; // J/mol*K
const float molar_mass_lpg = 44.1; // g/mol
const int table_size = 10;

// Tables
float pressure_table[table_size] = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9};
float volume_table[table_size] = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9};
float vacuum_table[table_size] = {-0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0};
float pressure_correction[table_size] = {1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1};
float flow_rate_correction[table_size] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

// Sensors
float pressure_bar = 1.0;
float temperature_k = 300;
float pulse_width_ms = 0;
float injections_per_minute = 0;
float map_sensor_bar = 1.0;

// Results
float volume_m3 = 0;
float flow_rate_m3_s = 0;

float linearize_volume(float p, float t) {
    if (t <= 0) return 0;
    float best_val = volume_table[0];
    float min_err = 9999;
    for (int i = 0; i < table_size; i++) {
        float err = abs(p - pressure_table[i]);
        if (err < min_err) {
            min_err = err;
            best_val = volume_table[i];
        }
    }
    return best_val;
}

float predict_pressure(float map_s) {
    float vacuum = map_s - 1.0;
    float best_corr = 1.0;
    float min_err = 9999;
    for (int i = 0; i < table_size; i++) {
        float err = abs(vacuum - vacuum_table[i]);
        if (err < min_err) {
            min_err = err;
            best_corr = pressure_correction[i];
        }
    }
    return map_s + abs(vacuum * best_corr);
}

float estimate_flow_rate(float pw, float ipm) {
    float v = linearize_volume(pressure_bar, temperature_k);

    float best_corr = 1.0;
    float min_err = 9999;
    for (int i = 0; i < table_size; i++) {
        float err = abs(v - volume_table[i]);
        if (err < min_err) {
            min_err = err;
            best_corr = flow_rate_correction[i];
        }
    }

    // Hagen-Poiseuille based estimate (simplified)
    float r = (nozzle_diameter_mm / 2000.0);
    float flow_est = (pw / 1000.0) * (ipm / 60.0) * (PI * pow(r, 4) * pressure_bar * 100000.0) / (8.0 * (gas_constant_R / (molar_mass_lpg/1000.0)) * temperature_k);
    return flow_est * best_corr;
}

void setup() {
    Serial.begin(115200);
}

void loop() {
    pressure_bar = analogRead(A0) / 102.3;
    temperature_k = analogRead(A1) / 2.0 + 200; // Mock mapping
    pulse_width_ms = analogRead(A2) / 50.0;
    injections_per_minute = analogRead(A3) * 5.0;
    map_sensor_bar = analogRead(A4) / 1023.0;

    volume_m3 = linearize_volume(pressure_bar, temperature_k);
    pressure_bar = predict_pressure(map_sensor_bar);
    flow_rate_m3_s = estimate_flow_rate(pulse_width_ms, injections_per_minute);

    static unsigned long lp = 0;
    if (millis() - lp > 500) {
        Serial.print("Vol: "); Serial.print(volume_m3);
        Serial.print(" | Flow: "); Serial.println(flow_rate_m3_s, 6);
        lp = millis();
    }
}
