#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "../afr_correction_estimator.ino"

void test_initial_values() {
    std::cout << "Testing initial values..." << std::endl;
    mock_arduino_init();
    setup();
    assert(buffer_index == 0);
    assert(buffer_full == false);
}

void test_full_cycle() {
    std::cout << "Testing full buffer cycle..." << std::endl;
    mock_arduino_init();
    setup();

    // Fill buffer with a sine wave for lambda and injector
    for(int i = 0; i < BUFFER_SIZE; i++) {
        set_analog_read(lambdaPin, 512 + 200 * sin(i * 0.5));
        set_analog_read(injectorPin, 512 + 100 * sin(i * 0.5 + 0.1));
        loop();
    }

    std::cout << "AFR: " << afr << std::endl;
    std::cout << "Correlation: " << lambda_correlation << std::endl;

    assert(afr > 0);
    assert(std::abs(lambda_correlation) > 0.5); // Should be highly correlated
}

int main() {
    test_initial_values();
    test_full_cycle();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
