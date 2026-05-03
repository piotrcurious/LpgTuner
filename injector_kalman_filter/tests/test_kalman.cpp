#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <random>

#include "../injector_kalman_filter.ino"

void test_kalman_convergence() {
    std::cout << "Testing Enhanced Kalman filter convergence..." << std::endl;
    mock_arduino_init();
    setup_kalman();

    // target values corresponding to H * x
    float z[4] = {8.0, 207.1, 0.966, 23183.6};

    std::cout << "Initial x: [" << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << "]" << std::endl;

    for(int i=0; i<2000; i++) {
        predict(0.01);
        update(z);
    }

    std::cout << "Final x: [" << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << "]" << std::endl;

    assert(std::abs(x[0] - 8.0) < 0.5);
    assert(std::abs(x[1] - 14.2) < 0.5);
    assert(std::abs(x[2] - 20.0) < 1.0);
    assert(std::abs(x[3] - 231.8) < 5.0);
}

void test_kalman_noise_rejection() {
    std::cout << "Testing noise rejection with orthogonal updates..." << std::endl;
    mock_arduino_init();
    setup_kalman();

    float z_true[4] = {8.0, 207.1, 0.966, 23183.6};

    std::mt19937 gen(42);
    std::normal_distribution<float> d0(0, 1.0); // PW noise
    std::normal_distribution<float> d1(0, 10.0); // MAP noise
    std::normal_distribution<float> d2(0, 0.05); // Lambda noise
    std::normal_distribution<float> d3(0, 500.0); // RPM noise

    for(int i=0; i<2000; i++) {
        float z_noisy[4] = {
            z_true[0] + d0(gen),
            z_true[1] + d1(gen),
            z_true[2] + d2(gen),
            z_true[3] + d3(gen)
        };
        predict(0.01);
        update(z_noisy);
    }

    std::cout << "Final x (noisy): [" << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << "]" << std::endl;

    // Filter should still converge close to true values despite noise
    assert(std::abs(x[0] - 8.0) < 1.0);
    assert(std::abs(x[1] - 14.2) < 1.0);
    assert(std::abs(x[2] - 20.0) < 2.0);
    assert(std::abs(x[3] - 231.8) < 10.0);
}

int main() {
    test_kalman_convergence();
    test_kalman_noise_rejection();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
