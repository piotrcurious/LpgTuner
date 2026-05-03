#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "../afr_duty_estimator.ino"

void reset_globals() {
    o2_voltage = 0.45;
    inj_duty = 0;
    inj_duty_ema = 0;
    afr = 14.7;
    afr_ema = 14.7;
    afr_min = 20.0;
    afr_max = 0.0;
    rpm = 0;
    o2_warm = false;
    o2_transitions = 0;
    o2_state = false;
    o2_prev_state = false;
    o2_rise_time = 0;
    o2_last_period_us = 0;

    last_inj_fall_us = 0;
    last_inj_width_us = 0;
    last_inj_period_us = 0;
    inj_updated = false;

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

    unsigned long t = 1000000;

    auto cycle_o2 = [&](float high_ratio) {
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

    // Run a few cycles to stabilize O2
    for(int i=0; i<6; i++) cycle_o2(0.5);

    std::cout << "AFR after stoich cycles: " << afr_ema << " Warm: " << o2_warm << std::endl;
    assert(o2_warm == true);
    assert(std::abs(afr_ema - 14.7) < 0.5);

    // Injector Test: 3000 RPM, 10% Duty
    // 3000 RPM -> 50 Hz revs -> 25 Hz injections (4-stroke)
    // 25 Hz -> 40ms period
    // 10% Duty -> 4ms pulse width
    t = 10000000;
    auto inj_pulse = [&](unsigned long width_us, unsigned long period_us) {
        // Active (Falling edge)
        set_micros(t);
        set_digital_read(INJ_PIN, LOW);
        loop();
        t += width_us;

        // Inactive (Rising edge)
        set_micros(t);
        set_digital_read(INJ_PIN, HIGH);
        loop();
        t += (period_us - width_us);
    };

    for(int i=0; i<50; i++) {
        inj_pulse(4000, 40000);
    }

    std::cout << "Injector Duty: " << inj_duty_ema * 100 << "%" << std::endl;
    std::cout << "RPM: " << rpm << std::endl;
    assert(std::abs(inj_duty_ema - 0.1) < 0.05);
    assert(std::abs(rpm - 3000) < 10);
}

void test_edge_cases() {
    std::cout << "Testing edge cases..." << std::endl;
    mock_arduino_init();
    reset_globals();
    set_analog_read(O2_PIN, (0.1/5.0)*1023);
    set_digital_read(INJ_PIN, HIGH);
    setup();

    // Engine stop
    set_micros(20000000);
    loop();
    std::cout << "RPM after stop: " << rpm << std::endl;
    assert(rpm == 0);
}

int main() {
    test_signals();
    test_edge_cases();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
