#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

// Include the .ino content
#include "../simple_lambda.ino"

void test_initial_values() {
    std::cout << "Testing initial values..." << std::endl;
    mock_arduino_init();
    setup();
    assert(std::abs(AFRPrev - 14.7) < 0.01);
}

void test_afr_mapping() {
    std::cout << "Testing AFR mapping..." << std::endl;
    mock_arduino_init();
    setup();

    // Test Lean (0.1V)
    set_analog_read(lambdaPin, (0.1 / 5.0) * 1023);
    loop();
    // 0.1V should map to 15.435
    std::cout << "Filtered AFR: " << AFRFiltered << std::endl;
    assert(AFRFiltered > 14.7);

    // Run multiple loops to stabilize
    for(int i=0; i<20; i++) loop();
    std::cout << "Stabilized Filtered AFR: " << AFRFiltered << std::endl;
    assert(std::abs(AFRFiltered - 15.435) < 0.1);

    // Test Rich (0.9V)
    set_analog_read(lambdaPin, (0.9 / 5.0) * 1023);
    for(int i=0; i<20; i++) loop();
    std::cout << "Stabilized Rich AFR: " << AFRFiltered << std::endl;
    assert(std::abs(AFRFiltered - 13.965) < 0.1);
}

void test_oscillation() {
    std::cout << "Testing oscillation..." << std::endl;
    mock_arduino_init();
    setup();

    unsigned long time_ms = 0;
    set_millis(time_ms);

    // 1. Establish Rich baseline
    set_analog_read(lambdaPin, (0.9 / 5.0) * 1023);
    for(int i=0; i<20; i++) {
        loop();
        time_ms += 10;
        set_millis(time_ms);
    }
    std::cout << "Baseline Rich AFR: " << AFRFiltered << " (Should be < 14.7)" << std::endl;

    // 2. Rising Edge: Cross from Rich to Lean (0.1V)
    set_analog_read(lambdaPin, (0.1 / 5.0) * 1023);
    for(int i=0; i<5; i++) { // cross it
        loop();
        time_ms += 10;
        set_millis(time_ms);
    }
    std::cout << "After Rising Edge AFR: " << AFRFiltered << std::endl;
    unsigned long t0 = startTime;
    std::cout << "startTime: " << t0 << std::endl;

    // 3. Falling Edge: Cross from Lean to Rich (0.9V)
    // Wait some time
    for(int i=0; i<50; i++) {
        loop();
        time_ms += 10;
        set_millis(time_ms);
    }
    set_analog_read(lambdaPin, (0.9 / 5.0) * 1023);
    for(int i=0; i<5; i++) {
        loop();
        time_ms += 10;
        set_millis(time_ms);
    }
    std::cout << "After Falling Edge AFR: " << AFRFiltered << std::endl;
    std::cout << "highTime: " << highTime << std::endl;

    // 4. Rising Edge again
    for(int i=0; i<50; i++) {
        loop();
        time_ms += 10;
        set_millis(time_ms);
    }
    set_analog_read(lambdaPin, (0.1 / 5.0) * 1023);
    for(int i=0; i<5; i++) {
        loop();
        time_ms += 10;
        set_millis(time_ms);
    }

    std::cout << "Final Frequency: " << frequency << " Hz" << std::endl;
    std::cout << "Final Period: " << period << " s" << std::endl;
    std::cout << "Final Duty Cycle: " << dutyCycle * 100 << " %" << std::endl;

    assert(period > 0);
}

int main() {
    test_initial_values();
    test_afr_mapping();
    test_oscillation();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
