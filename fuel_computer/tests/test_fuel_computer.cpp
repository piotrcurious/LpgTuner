#include <iostream>
#include <cassert>
#include <cmath>
#include "mock_arduino.h"

// Hack to include the .ino file without name collisions for setup/loop
#define setup arduino_setup
#define loop arduino_loop
#include "../fuel_computer.ino"
#undef setup
#undef loop

// Include others. They don't have setup/loop so no need for hacks.
#include "../Lambda_computer1.ino"
#include "../fuel_computer2.ino"

bool is_close(float a, float b, float epsilon = 0.001f) {
    return std::abs(a - b) < epsilon;
}

void test_fuel_computer_ino() {
    std::cout << "Testing fuel_computer.ino..." << std::endl;

    // calculateFuelConsumption_LitersPerHour
    assert(is_close(calculateFuelConsumption_LitersPerHour(5.0, 0), 0.0));
    assert(is_close(calculateFuelConsumption_LitersPerHour(5.0, 2500), 3.125));
    assert(is_close(calculateFuelConsumption_LitersPerHour(10.0, 6000), 15.0));

    // calculateFuelEconomy_LitersPer100km
    assert(is_close(calculateFuelEconomy_LitersPer100km(5.0, 2500, 0), 0.0));
    assert(is_close(calculateFuelEconomy_LitersPer100km(5.0, 2500, 80.0), 3.90625));

    std::cout << "  Passed!" << std::endl;
}

void test_Lambda_computer1_ino() {
    std::cout << "Testing Lambda_computer1.ino (fuelComputer)..." << std::endl;

    assert(is_close(fuelComputer(5.0, 2500), 3.125));
    assert(is_close(fuelComputer(5.0, 0), 0.0));

    std::cout << "  Passed!" << std::endl;
}

void test_fuel_computer2_ino() {
    std::cout << "Testing fuel_computer2.ino (fuelComputer2)..." << std::endl;

    assert(is_close(fuelComputer2(5.0, 2500, 80.0), 3.90625));
    assert(is_close(fuelComputer2(5.0, 2500, 0), 0.0));

    std::cout << "  Passed!" << std::endl;
}

int main() {
    test_fuel_computer_ino();
    test_Lambda_computer1_ino();
    test_fuel_computer2_ino();

    std::cout << "\nAll tests passed successfully!" << std::endl;
    return 0;
}
