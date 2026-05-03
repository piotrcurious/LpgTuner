#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <random>

#include "../injector_kalman_filter.ino"

void test_kalman_convergence() {
    std::cout << "Testing EKF convergence..." << std::endl;
    mock_arduino_init();
    setup_kalman();

    // Physically consistent target:
    // x = [8.0, 14.2, 20.0, 248.45]
    // PW = 8.0
    // AFR = 14.2
    // Flow = (0.5 * 8.0) / 0.2 = 20.0
    // Torque = (1.2 * 20.0 * (14.7 / 14.2)) / 0.1 = 248.45

    // Measurements:
    // z[0] = 8.0
    // z[1] = 0.5 * 14.2 + 10.0 * 20.0 = 207.1
    // z[2] = 14.2 / 14.7 = 0.965986
    // z[3] = 100.0 * 248.45 = 24845.0

    float z[4] = {8.0, 207.1, 0.965986, 24845.0};

    for(int i=0; i<5000; i++) {
        predict(0.01);
        update(z);
    }

    std::cout << "Final x: [" << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << "]" << std::endl;

    assert(std::abs(x[0] - 8.0) < 0.5);
    assert(std::abs(x[1] - 14.2) < 0.5);
    assert(std::abs(x[2] - 20.0) < 1.0);
    assert(std::abs(x[3] - 248.45) < 5.0);
}

void test_outlier_rejection() {
    std::cout << "Testing outlier rejection..." << std::endl;
    mock_arduino_init();
    setup_kalman();

    float z_true[4] = {8.0, 207.1, 0.965986, 24845.0};

    for(int i=0; i<2000; i++) {
        predict(0.01);
        update(z_true);
    }

    float x_before = x[0];
    float z_outlier[4] = {100.0, 207.1, 0.965986, 24845.0};
    predict(0.01);
    update(z_outlier);

    std::cout << "x[0] after outlier: " << x[0] << " (before: " << x_before << ")" << std::endl;
    assert(std::abs(x[0] - x_before) < 0.1);
}

void test_dynamic_response() {
    std::cout << "Testing dynamic response with adaptive Q..." << std::endl;
    mock_arduino_init();
    setup_kalman();

    // x = [5, 14.7, 12.5, 120.0]
    // z[0] = 5.0
    // z[1] = 0.5 * 14.7 + 10 * 12.5 = 132.35
    // z[2] = 1.0
    // z[3] = 12000.0
    float z_initial[4] = {5.0, 132.35, 1.0, 12000.0};
    for(int i=0; i<2000; i++) {
        predict(0.01);
        update(z_initial);
    }

    std::cout << "Steady state x: [" << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << "]" << std::endl;

    // x = [10, 14.7, 25.0, 240.0]
    float z_step[4] = {10.0, 257.35, 1.0, 24000.0};
    int steps = 0;
    while(std::abs(x[0] - 10.0) > 0.5 && steps < 500) {
        predict(0.01);
        update(z_step);
        steps++;
    }

    std::cout << "Reached target in " << steps << " steps." << std::endl;
    assert(steps < 100);
}

void test_sensor_glitch_series() {
    std::cout << "Testing resilience to sensor glitch series..." << std::endl;
    mock_arduino_init();
    setup_kalman();

    float z_true[4] = {8.0, 207.1, 0.965986, 24845.0};
    for(int i=0; i<2000; i++) {
        predict(0.01);
        update(z_true);
    }

    float x_ref = x[0];

    // Series of glitches on Throttle
    for(int i=0; i<10; i++) {
        float z_glitch[4] = {100.0, 207.1, 0.965986, 24845.0};
        predict(0.01);
        update(z_glitch);
    }

    std::cout << "x[0] after 10 glitches: " << x[0] << " (ref: " << x_ref << ")" << std::endl;
    assert(std::abs(x[0] - x_ref) < 0.2);
}

int main() {
    test_kalman_convergence();
    test_outlier_rejection();
    test_dynamic_response();
    test_sensor_glitch_series();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
