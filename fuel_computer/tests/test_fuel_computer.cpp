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
    // Standard: 2500 RPM, 5ms pulse, 4 cylinders, 0.5 L/min flow
    // injections_per_minute = (2500 / 2) * 4 = 5000
    // total_duration_ms = 5000 * 5 = 25000 ms
    // duration_min = 25000 / 60000 = 0.41666... min
    // L_per_min = 0.41666... * 0.5 = 0.208333...
    // L_per_hour = 0.208333... * 60 = 12.5
    assert(is_close(calculateFuelConsumption_LitersPerHour(5.0, 2500), 12.5));

    // High RPM: 6000 RPM, 10ms pulse
    // injections_per_minute = (6000 / 2) * 4 = 12000
    // total_duration_ms = 12000 * 10 = 120000 ms (Note: this is 100% duty cycle for the engine overall, but 50% per cylinder)
    // duration_min = 120000 / 60000 = 2 min
    // L_per_min = 2 * 0.5 = 1.0
    // L_per_hour = 1.0 * 60 = 60.0
    assert(is_close(calculateFuelConsumption_LitersPerHour(10.0, 6000), 60.0));

    // calculateFuelEconomy_LitersPer100km
    // consumption = 12.5 L/h, speed = 80 km/h
    // economy = (12.5 / 80) * 100 = 15.625
    assert(is_close(calculateFuelEconomy_LitersPer100km(5.0, 2500, 80.0), 15.625));

    std::cout << "  Passed!" << std::endl;
}

void test_trip_statistics() {
    std::cout << "Testing trip statistics..." << std::endl;

    // Reset globals
    totalFuelConsumed_L = 0.0f;
    fuelCompensation = 0.0f;
    totalDistance_km = 0.0f;
    distanceCompensation = 0.0f;
    tripStartTime_ms = millis();

    // Simulate 1 hour trip at 100 km/h with 10 L/h consumption
    float speed = 100.0f;
    float consumption = 10.0f;
    float deltaTime = 1.0f; // 1 second steps

    for (int i = 0; i < 3600; ++i) {
        accumulateFuel(consumption, deltaTime);
        accumulateDistance(speed, deltaTime);
        // We don't update mock millis manually here because it's static in mock_arduino.cpp
        // and we are just testing the accumulation logic functions directly.
    }

    // After 1 hour:
    // Distance should be 100 km
    // Fuel should be 10 L
    assert(is_close(totalDistance_km, 100.0f));
    assert(is_close(totalFuelConsumed_L, 10.0f));

    // Averages (using the logic from loop())
    float averageEconomy = (totalDistance_km > 0) ? (totalFuelConsumed_L / totalDistance_km * 100.0f) : 0;
    assert(is_close(averageEconomy, 10.0f));

    std::cout << "  Passed!" << std::endl;
}

void test_Lambda_computer1_ino() {
    std::cout << "Testing Lambda_computer1.ino (fuelComputer)..." << std::endl;

    // Logic should be identical to fuel_computer.ino
    assert(is_close(fuelComputer(5.0, 2500), 12.5));
    assert(is_close(fuelComputer(10.0, 6000), 60.0));

    std::cout << "  Passed!" << std::endl;
}

void test_fuel_computer2_ino() {
    std::cout << "Testing fuel_computer2.ino (fuelComputer2)..." << std::endl;

    assert(is_close(fuelComputer2(5.0, 2500, 80.0), 15.625));

    std::cout << "  Passed!" << std::endl;
}

void test_extreme_values() {
    std::cout << "Testing extreme values..." << std::endl;

    // Max RPM, Max Pulse
    // 8000 RPM -> 4000 cycles/min/cyl -> 15ms pulse
    // total_duration = 4000 * 15 * 4 = 240000 ms/min
    // duration_min = 240000 / 60000 = 4 min
    // L/h = 4 * 0.5 * 60 = 120.0
    assert(is_close(calculateFuelConsumption_LitersPerHour(15.0, 8000), 120.0));

    // Very low RPM
    assert(is_close(calculateFuelConsumption_LitersPerHour(1.0, 100), 0.1));

    std::cout << "  Passed!" << std::endl;
}

void test_kahan_summation() {
    std::cout << "Testing Kahan summation stability..." << std::endl;

    // Reset globals
    totalFuelConsumed_L = 0.0f;
    fuelCompensation = 0.0f;

    // We'll simulate adding a very small increment many times.
    // Let's say 0.000001 L added 1,000,000 times.
    // Total should be exactly 1.0.

    float smallIncrement_L_h = 0.0036f; // 0.0036 L/h = 0.000001 L/s
    float deltaTime_s = 1.0f;

    float naiveSum = 0.0f;
    for (int i = 0; i < 1000000; ++i) {
        accumulateFuel(smallIncrement_L_h, deltaTime_s);
        naiveSum += (smallIncrement_L_h / 3600.0f) * deltaTime_s;
    }

    std::cout << "  Naive sum: " << std::fixed << std::setprecision(10) << naiveSum << std::endl;
    std::cout << "  Kahan sum: " << std::fixed << std::setprecision(10) << totalFuelConsumed_L << std::endl;

    // In many environments, 32-bit float naive sum will start losing precision significantly
    // when the sum becomes much larger than the increment.
    // 1.0 is ~2^20 times larger than 0.000001.
    // 32-bit float has ~7 decimal digits of precision.

    // We expect Kahan sum to be much closer to 1.0
    assert(is_close(totalFuelConsumed_L, 1.0f, 0.0001f));

    // Naive sum often fails to reach exactly 1.0 in this scenario
    // (though it depends on the exact float representation and rounding mode)

    std::cout << "  Passed!" << std::endl;
}

int main() {
    test_fuel_computer_ino();
    test_Lambda_computer1_ino();
    test_fuel_computer2_ino();
    test_extreme_values();
    test_kahan_summation();
    test_trip_statistics();

    std::cout << "\nAll enhanced tests passed successfully!" << std::endl;
    return 0;
}
