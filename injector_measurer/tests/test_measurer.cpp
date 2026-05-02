#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "../injector_measurer.ino"

void test_measurement() {
    std::cout << "Testing injector measurement..." << std::endl;
    mock_arduino_init();

    // Pulse 1: 5ms
    set_micros(1000);
    set_digital_read(injectorPin, HIGH);
    injectorISR();
    set_micros(6000);
    set_digital_read(injectorPin, LOW);
    injectorISR();

    loop();
    std::cout << "Smoothed PW: " << smoothedPulseWidthMs << std::endl;
    assert(smoothedPulseWidthMs > 0);

    // Simulate many 5ms pulses
    for(int i=0; i<100; i++) {
        set_micros(10000 + i*10000);
        set_digital_read(injectorPin, HIGH);
        injectorISR();
        set_micros(15000 + i*10000);
        set_digital_read(injectorPin, LOW);
        injectorISR();
        loop();
    }

    std::cout << "Final Smoothed PW: " << smoothedPulseWidthMs << std::endl;
    assert(std::abs(smoothedPulseWidthMs - 5.0) < 0.1);
}

int main() {
    test_measurement();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
