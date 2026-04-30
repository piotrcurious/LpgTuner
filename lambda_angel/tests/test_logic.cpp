#include "mock_arduino.h"
#include "../lambda_angel_tft_map3/CommonLogic.h"
#include <cassert>
#include <iostream>

void test_fmap() {
    assert(std::abs(fmap(5, 0, 10, 0, 100) - 50.0) < 0.001);
    assert(std::abs(fmap(1.65, 0, 3.3, 0, 1.0) - 0.5) < 0.001);
    std::cout << "test_fmap passed" << std::endl;
}

void test_bins() {
    assert(getRPMBin(500) == 0);
    assert(getRPMBin(1500) == 1);
    assert(getRPMBin(9500) == 9);
    assert(getLoadBin(30) == 0);
    assert(getLoadBin(240) == 7);
    assert(getRPMBin(1000) == 1);
    assert(getLoadBin(40) == 1);
    // Boundary and OOB
    assert(getRPMBin(-100) == 0);
    assert(getRPMBin(12000) == 9);
    assert(getLoadBin(10) == 0);
    assert(getLoadBin(300) == 7);
    std::cout << "test_bins passed" << std::endl;
}

void test_units() {
    assert(std::abs(voltageToKpa(0.5) - 20.0) < 0.001);
    assert(std::abs(voltageToKpa(4.5) - 250.0) < 0.001);
    std::cout << "test_units passed" << std::endl;
}

void test_heatmap_averaging() {
    float mapVal = 0;
    int count = 0;
    updateMapSample(mapVal, 500.0, count);
    assert(count == 1);
    assert(mapVal == 500.0);
    updateMapSample(mapVal, 600.0, count);
    assert(count == 2);
    assert(std::abs(mapVal - 510.0) < 0.001);
    std::cout << "test_heatmap_averaging passed" << std::endl;
}

int main() {
    test_fmap();
    test_bins();
    test_units();
    test_heatmap_averaging();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
