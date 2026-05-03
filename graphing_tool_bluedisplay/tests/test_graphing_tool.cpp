#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "../graphing_tool_bluedisplay.ino"

void test_sensor_readings() {
    std::cout << "Testing sensor readings..." << std::endl;
    mock_arduino_init();

    set_analog_read(throttlePin, 512); // ~50%
    set_analog_read(mapPin, 1023); // 100%
    set_analog_read(lambdaPin, 512);

    // Simulate some cam pulses (6000 RPM = 100 Hz = 10ms period)
    set_micros(100000);
    camISR();
    set_micros(110000);
    camISR();

    // Simulate injector pulse (2ms) - Active LOW
    set_micros(110000);
    set_digital_read(injectorPin, LOW); // ON
    injectorISR();
    set_micros(112000);
    set_digital_read(injectorPin, HIGH); // OFF
    injectorISR();

    readSensors();

    std::cout << "RPM: " << rpm << std::endl;
    std::cout << "PulseWidth: " << pulseWidthMs << " ms" << std::endl;
    std::cout << "Throttle: " << throttle << " %" << std::endl;

    assert(std::abs(rpm - 6000) < 1);
    assert(std::abs(pulseWidthMs - 2.0) < 0.1);
    assert(std::abs(throttle - 50.0) < 1.0);
}

void test_timeouts() {
    std::cout << "Testing timeouts..." << std::endl;
    mock_arduino_init();

    // Set initial state
    set_micros(100000);
    camISR();
    set_micros(110000);
    camISR();

    set_digital_read(injectorPin, LOW);
    injectorISR();
    set_micros(112000);
    set_digital_read(injectorPin, HIGH);
    injectorISR();

    readSensors();
    assert(rpm > 0);
    assert(pulseWidthMs > 0);

    // Wait for timeout (0.6s)
    set_micros(712000);
    readSensors();

    std::cout << "RPM after timeout: " << rpm << std::endl;
    std::cout << "PulseWidth after timeout: " << pulseWidthMs << std::endl;

    assert(rpm == 0);
    assert(pulseWidthMs == 0);
}

int main() {
    test_sensor_readings();
    test_timeouts();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
