#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "../injector_kalman_filter.ino"

void test_kalman_convergence() {
    std::cout << "Testing Kalman filter convergence..." << std::endl;
    mock_arduino_init();
    setup_kalman();

    float z[4] = {8.0, 14.2, 0.6, 2000.0}; // target values

    std::cout << "Initial x: [" << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << "]" << std::endl;

    for(int i=0; i<100; i++) {
        predict(0.01);
        update(z);
    }

    std::cout << "Final x: [" << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << "]" << std::endl;

    assert(std::abs(x[0] - 8.0) < 0.1);
    assert(std::abs(x[1] - 14.2) < 0.1);
}

int main() {
    test_kalman_convergence();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
