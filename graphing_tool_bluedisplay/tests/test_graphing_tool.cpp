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

    // Simulate injector pulse (2ms)
    set_micros(110000);
    set_digital_read(injectorPin, HIGH);
    injectorISR();
    set_micros(112000);
    set_digital_read(injectorPin, LOW);
    injectorISR();

    readSensors();

    std::cout << "RPM: " << rpm << std::endl;
    std::cout << "PulseWidth: " << pulseWidthMs << " ms" << std::endl;
    std::cout << "Throttle: " << throttle << " %" << std::endl;

    assert(std::abs(rpm - 6000) < 1);
    assert(std::abs(pulseWidthMs - 2.0) < 0.1);
    assert(std::abs(throttle - 50.0) < 1.0);
}

int main() {
    test_sensor_readings();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
