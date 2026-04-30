#include "mock_arduino.h"
#include "../lambda_angel_tft_map3/CommonLogic.h"
#include <cassert>
#include <iostream>
#include <vector>

void test_fmap() {
    assert(std::abs(fmap(5, 0, 10, 0, 100) - 50.0) < 0.001);
    assert(std::abs(fmap(1.65, 0, 3.3, 0, 1.0) - 0.5) < 0.001);
    assert(fmap(5, 10, 10, 0, 100) == 0.0);
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
    assert(getRPMBin(-100) == 0);
    assert(getRPMBin(12000) == 9);
    assert(getLoadBin(10) == 0);
    assert(getLoadBin(300) == 7);
    std::cout << "test_bins passed" << std::endl;
}

void test_lowpass() {
    float val = 100.0f;
    val = lowPass(val, 200.0f, 0.1f);
    assert(std::abs(val - 110.0f) < 0.001);
    std::cout << "test_lowpass passed" << std::endl;
}

void test_units() {
    assert(std::abs(voltageToKpa(0.5) - 20.0) < 0.001);
    assert(std::abs(voltageToKpa(4.5) - 250.0) < 0.001);
    assert(voltageToKpa(0.0) == 20.0);
    assert(voltageToKpa(5.0) == 250.0);
    std::cout << "test_units passed" << std::endl;
}

void test_lambda_zones() {
    assert(getLambdaZone(1.10f) == LAMBDA_LEAN);
    assert(getLambdaZone(1.00f) == LAMBDA_STOICH);
    assert(getLambdaZone(0.90f) == LAMBDA_RICH);
    assert(getLambdaZone(1.04f) == LAMBDA_STOICH);
    assert(getLambdaZone(0.96f) == LAMBDA_STOICH);
    std::cout << "test_lambda_zones passed" << std::endl;
}

void test_heatmap_simulation() {
    float tempMap[RPM_BINS][LOAD_BINS] = {0};
    int sampleCount[RPM_BINS][LOAD_BINS] = {0};
    int r = getRPMBin(2500);
    int l = getLoadBin(60);
    std::vector<float> readings = {500, 510, 520, 530, 540, 550, 560, 570, 580, 590};
    for (float val : readings) {
        updateMapSample(tempMap[r][l], val, sampleCount[r][l], 0.1f);
    }
    assert(sampleCount[r][l] == 10);
    assert(tempMap[r][l] > 500.0);
    assert(tempMap[r][l] < 545.0);
    std::cout << "test_heatmap_simulation passed" << std::endl;
}

int main() {
    test_fmap();
    test_bins();
    test_lowpass();
    test_units();
    test_lambda_zones();
    test_heatmap_simulation();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
