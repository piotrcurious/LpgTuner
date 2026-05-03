#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

// Define variables as extern to reset them between tests if needed,
// but including .ino defines them.
#include "../afr_duty_estimator.ino"

void reset_globals() {
    o2_voltage = 0.45;
    inj_duty = 0;
    inj_duty_ema = 0;
    afr = 14.7;
    afr_ema = 14.7;
    afr_min = 20.0;
    afr_max = 0.0;
    o2_state = false;
    o2_prev_state = false;
    inj_state = false;
    inj_prev_state = false;
    o2_rise_time = 0;
    o2_last_period_us = 0;
    inj_fall_time = 0;
    inj_last_period_us = 0;
    afr_buffer_index = 0;
    for(int i=0; i<GRAPH_WIDTH; i++) afr_buffer[i] = 14.7;
}

void test_signals() {
    std::cout << "Testing O2 and Injector signals..." << std::endl;
    mock_arduino_init();
    reset_globals();

    // Initial state: Lean (digitalRead(INJ_PIN) == HIGH is inactive)
    set_analog_read(O2_PIN, (0.1/5.0)*1023);
    set_digital_read(INJ_PIN, HIGH);
    setup();

    // 1Hz oscillation, 80% Rich (0.8s High, 0.2s Low)
    unsigned long t = 1000000;

    auto cycle = [&](float high_ratio) {
        unsigned long period = 1000000;
        unsigned long high_time = period * high_ratio;
        unsigned long low_time = period - high_time;

        // High
        set_analog_read(O2_PIN, (0.9/5.0)*1023);
        set_micros(t); loop();
        t += high_time;

        // Low
        set_analog_read(O2_PIN, (0.1/5.0)*1023);
        set_micros(t); loop();
        t += low_time;
    };

    // Run a few cycles to stabilize
    cycle(0.5); // Stoich
    cycle(0.5);

    std::cout << "AFR after stoich cycles: " << afr_ema << std::endl;
    assert(std::abs(afr_ema - 14.7) < 0.5);

    cycle(0.8); // Rich (80% high time)
    std::cout << "AFR after 80% Rich: " << afr_ema << std::endl;
    assert(afr_ema < 14.7);

    cycle(0.2); // Lean (20% high time)
    std::cout << "AFR after 20% Lean: " << afr_ema << std::endl;
    assert(afr_ema > 12.0); // Should be moving back up

    // Injector Test: 100Hz, 2ms pulse (20%)
    t = 10000000;
    auto inj_pulse = [&](unsigned long width_us) {
        unsigned long period = 10000; // 100Hz

        // Active (Falling edge)
        set_digital_read(INJ_PIN, LOW);
        set_micros(t); loop();
        t += width_us;

        // Inactive (Rising edge)
        set_digital_read(INJ_PIN, HIGH);
        set_micros(t); loop();
        t += (period - width_us);
    };

    // Need more pulses to stabilize EMA
    for(int i=0; i<50; i++) {
        inj_pulse(2000);
    }

    std::cout << "Injector Duty: " << inj_duty_ema * 100 << "%" << std::endl;
    assert(std::abs(inj_duty_ema - 0.2) < 0.05);
}

void test_edge_cases() {
    std::cout << "Testing edge cases..." << std::endl;
    mock_arduino_init();
    reset_globals();
    set_analog_read(O2_PIN, (0.1/5.0)*1023);
    set_digital_read(INJ_PIN, HIGH);
    setup();

    // No transitions: AFR should stay default
    for(int i=0; i<10; i++) {
        set_micros(1000000 + i * 100000);
        loop();
    }
    std::cout << "AFR No transitions: " << afr_ema << std::endl;
    assert(std::abs(afr_ema - 14.7) < 0.001);

    // Stuck Rich
    set_analog_read(O2_PIN, (0.9/5.0)*1023);
    for(int i=0; i<10; i++) {
        set_micros(3000000 + i * 100000);
        loop();
    }
    // One transition happened (Low to High), but high-to-low never happened, so duty not updated
    std::cout << "AFR Stuck Rich: " << afr_ema << std::endl;
    assert(std::abs(afr_ema - 14.7) < 0.001);
}

int main() {
    test_signals();
    test_edge_cases();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
