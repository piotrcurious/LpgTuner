#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "../afr_duty_estimator.ino"

void test_signals() {
    std::cout << "Testing O2 and Injector signals..." << std::endl;
    mock_arduino_init();
    setup();

    // Simulate O2 oscillation at 1Hz (1000ms period)
    // 0.9V for 500ms, 0.1V for 500ms
    unsigned long t = 1000;

    // Cycle 1
    set_analog_read(O2_PIN, (0.9/5.0)*1023); // High
    set_micros(t); loop();
    t += 500000;
    set_analog_read(O2_PIN, (0.1/5.0)*1023); // Low
    set_micros(t); loop();
    t += 500000;

    // Cycle 2
    set_analog_read(O2_PIN, (0.9/5.0)*1023); // High
    set_micros(t); loop();
    t += 500000;
    set_analog_read(O2_PIN, (0.1/5.0)*1023); // Low
    set_micros(t); loop();

    std::cout << "Estimated AFR: " << afr << std::endl;
    assert(std::abs(afr - 14.7) < 1.0);

    // Simulate Injector at 100Hz (10ms period), 2ms pulse width (20% duty)
    t = 10000;
    set_micros(t);
    set_digital_read(INJ_PIN, HIGH); // Inactive
    loop();

    t += 10000;
    set_micros(t);
    set_digital_read(INJ_PIN, LOW); // Active
    loop();

    t += 2000;
    set_micros(t);
    set_digital_read(INJ_PIN, HIGH); // Inactive
    loop();

    t += 8000;
    set_micros(t);
    set_digital_read(INJ_PIN, LOW); // Active
    loop();

    t += 2000;
    set_micros(t);
    set_digital_read(INJ_PIN, HIGH); // Inactive
    loop();

    std::cout << "Injector Duty: " << inj_duty * 100 << " %" << std::endl;
    assert(std::abs(inj_duty - 0.2) < 0.05);
}

int main() {
    test_signals();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
