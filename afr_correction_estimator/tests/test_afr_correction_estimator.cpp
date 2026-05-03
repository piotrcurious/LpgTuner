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

void test_noisy_signal() {
    std::cout << "Testing noisy signal with hysteresis..." << std::endl;
    mock_arduino_init();
    setup();

    // Signal with small noise (within hysteresis)
    for(int i = 0; i < BUFFER_SIZE; i++) {
        // Base sine wave + noise that should be filtered by hysteresis
        int base = 512 + 200 * sin(i * 0.5);
        int noise = (i % 2 == 0) ? 10 : -10; // +/- 10 is less than HYSTERESIS (20)
        set_analog_read(LP_PIN, base + noise);
        set_analog_read(IJ_PIN, 512);
        set_millis(i * 10);
        loop();
    }

    std::cout << "Peak Count: " << peak_count << " | Valley Count: " << valley_count << std::endl;
    std::cout << "Avg Peak: " << avg_peak << " | Avg Valley: " << avg_valley << std::endl;

    // With 0.5 frequency and 100 samples, we should have about 8 peaks/valleys
    // If hysteresis works, we shouldn't have hundreds of peaks from noise.
    assert(peak_count > 0 && peak_count < 20);
    assert(valley_count > 0 && valley_count < 20);
}

void test_ema_filtering() {
    std::cout << "Testing EMA filtering..." << std::endl;
    mock_arduino_init();
    setup();

    // Cycle 1: Constant rich
    for(int i = 0; i < BUFFER_SIZE; i++) {
        set_analog_read(LP_PIN, 800);
        set_analog_read(IJ_PIN, 512);
        set_millis(i * 10);
        loop();
    }
    float afr1 = afr;
    float afr_ema1 = afr_filtered;

    // Cycle 2: Constant lean
    for(int i = 0; i < BUFFER_SIZE; i++) {
        set_analog_read(LP_PIN, 100);
        set_analog_read(IJ_PIN, 512);
        set_millis((BUFFER_SIZE + i) * 10);
        loop();
    }
    float afr2 = afr;
    float afr_ema2 = afr_filtered;

    std::cout << "C1: AFR=" << afr1 << " EMA=" << afr_ema1 << std::endl;
    std::cout << "C2: AFR=" << afr2 << " EMA=" << afr_ema2 << std::endl;

    // EMA should be between the previous value and the new value
    assert(afr2 > afr1);
    assert(afr_ema2 > afr_ema1);
    assert(afr_ema2 < afr2); // EMA should lag behind the jump
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
    test_noisy_signal();
    test_ema_filtering();
    test_rich_condition();
    test_lean_condition();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
