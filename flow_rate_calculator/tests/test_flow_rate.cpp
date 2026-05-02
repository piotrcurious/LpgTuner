#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "../flow_rate_calculator.ino"

void test_calculations() {
    std::cout << "Testing LPG calculations..." << std::endl;
    mock_arduino_init();
    setup();

    gas_temperature_k = 300;
    gas_pressure_bar = 1.2;
    manifold_pressure_bar = 0.5;
    nozzle_diameter_mm = 2.0;
    injector_pulse_width_ms = 5.0;
    injections_per_minute = 2000;

    calculate_gas_density();
    calculate_flow_rate();

    std::cout << "Density: " << gas_density << " kg/m^3" << std::endl;
    std::cout << "Flow Rate: " << flow_rate_kg_s << " kg/s" << std::endl;

    assert(gas_density > 0);
    assert(flow_rate_kg_s > 0);
}

int main() {
    test_calculations();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
