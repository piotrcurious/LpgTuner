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
    // 100 samples in the buffer. 10ms per sample.
    for(int i = 0; i < BUFFER_SIZE; i++) {
        set_analog_read(LP_PIN, 512 + 200 * sin(i * 0.5));
        set_analog_read(IJ_PIN, 512 + 100 * sin(i * 0.5));
        set_millis(i * 10);
        loop();
    }

    std::cout << "AFR: " << afr << " | Corr: " << lambda_correlation << " | Lag: " << best_lag << std::endl;

    assert(afr > 10.0 && afr < 20.0);
    assert(std::abs(lambda_correlation) > 0.9);
    assert(best_lag == 0);
}

void test_lagged_correlation() {
    std::cout << "Testing lagged correlation..." << std::endl;
    mock_arduino_init();
    setup();

    int lag = 10;
    // Fill buffer
    for(int i = 0; i < BUFFER_SIZE; i++) {
        set_analog_read(LP_PIN, 512 + 200 * sin(i * 0.3));

        int injector_val;
        if (i >= lag) {
            injector_val = 512 + 100 * sin((i - lag) * 0.3);
        } else {
            injector_val = 512;
        }
        set_analog_read(IJ_PIN, injector_val);
        set_millis(i * 10);
        loop();
    }

    std::cout << "AFR: " << afr << " | Corr: " << lambda_correlation << " | Lag: " << best_lag << std::endl;

    assert(std::abs(lambda_correlation) > 0.8);
    // Best lag might be slightly off due to the signal starting partway or discrete sampling
    assert(std::abs(best_lag - lag) <= 1);
}

void test_rich_condition() {
    std::cout << "Testing rich condition..." << std::endl;
    mock_arduino_init();
    setup();

    // Stuck high (rich) -> Voltage > 0.45V
    for(int i = 0; i < BUFFER_SIZE; i++) {
        set_analog_read(LP_PIN, 800);
        set_analog_read(IJ_PIN, 600);
        set_millis(i * 10);
        loop();
    }

    std::cout << "AFR: " << afr << " | Dev: " << deviation << std::endl;
    assert(afr < STOICH);
    assert(deviation < 0);
}

void test_lean_condition() {
    std::cout << "Testing lean condition..." << std::endl;
    mock_arduino_init();
    setup();

    // Stuck low (lean) -> Voltage < 0.45V
    for(int i = 0; i < BUFFER_SIZE; i++) {
        set_analog_read(LP_PIN, 50);
        set_analog_read(IJ_PIN, 400);
        set_millis(i * 10);
        loop();
    }

    std::cout << "AFR: " << afr << " | Dev: " << deviation << std::endl;
    assert(afr > STOICH);
    assert(deviation > 0);
}

int main() {
    test_initial_values();
    test_full_cycle();
    test_lagged_correlation();
    test_rich_condition();
    test_lean_condition();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
