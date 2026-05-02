#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "../flow_rate_estimator.ino"

void test_estimator() {
    std::cout << "Testing LPG estimator..." << std::endl;
    mock_arduino_init();

    pressure_bar = 1.2;
    temperature_k = 300;
    pulse_width_ms = 5.0;
    injections_per_minute = 2000;
    map_sensor_bar = 0.5;

    float v = linearize_volume(pressure_bar, temperature_k);
    float p = predict_pressure(map_sensor_bar);

    // Update pressure for flow rate calculation
    pressure_bar = p;
    float f = estimate_flow_rate(pulse_width_ms, injections_per_minute);

    std::cout << "Linearized Volume: " << v << std::endl;
    std::cout << "Predicted Pressure: " << p << std::endl;
    std::cout << "Flow Rate: " << f << " m^3/s" << std::endl;

    assert(v > 0);
    assert(p > 0.4);
    assert(f > 0);
}

int main() {
    test_estimator();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
