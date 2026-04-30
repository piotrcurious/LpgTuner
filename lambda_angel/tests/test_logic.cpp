#include "mock_arduino.h"
#include "../lambda_angel/CommonLogic.h"
#include <cassert>
#include <iostream>

// Helper to simulate heat map update in test
void updateHeatMapTest(float rpm, float map, float temp, float lambda,
                       float tempMap[RPM_BINS][LOAD_BINS],
                       float lambdaMap[RPM_BINS][LOAD_BINS],
                       int sampleCount[RPM_BINS][LOAD_BINS]) {
  if (rpm > 500) {
    int r = getRPMBin(rpm);
    int l = getLoadBin(map);
    if (temp > 0 && temp < 1000 && lambda > 0 && lambda < 5.0) {
      if (sampleCount[r][l] == 0) {
        tempMap[r][l] = temp;
        lambdaMap[r][l] = lambda;
        sampleCount[r][l] = 1;
      } else {
        float alpha = 0.1;
        tempMap[r][l] = tempMap[r][l] * (1 - alpha) + temp * alpha;
        lambdaMap[r][l] = lambdaMap[r][l] * (1 - alpha) + lambda * alpha;
        sampleCount[r][l]++;
      }
    }
  }
}

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
    std::cout << "test_bins passed" << std::endl;
}

void test_units() {
    assert(std::abs(voltageToKpa(0.5) - 20.0) < 0.001);
    assert(std::abs(voltageToKpa(4.5) - 250.0) < 0.001);
    std::cout << "test_units passed" << std::endl;
}

void test_heatmap_averaging() {
    float tempMap[RPM_BINS][LOAD_BINS] = {0};
    float lambdaMap[RPM_BINS][LOAD_BINS] = {0};
    int sampleCount[RPM_BINS][LOAD_BINS] = {0};

    // First sample
    updateHeatMapTest(2000, 100, 500.0, 1.0, tempMap, lambdaMap, sampleCount);
    assert(sampleCount[2][4] == 1);
    assert(tempMap[2][4] == 500.0);

    // Second sample (averaging)
    updateHeatMapTest(2000, 100, 600.0, 1.2, tempMap, lambdaMap, sampleCount);
    assert(sampleCount[2][4] == 2);
    // 500 * 0.9 + 600 * 0.1 = 450 + 60 = 510
    assert(std::abs(tempMap[2][4] - 510.0) < 0.001);
    assert(std::abs(lambdaMap[2][4] - 1.02) < 0.001);

    // Gating check (engine off)
    updateHeatMapTest(100, 100, 700.0, 1.5, tempMap, lambdaMap, sampleCount);
    assert(sampleCount[2][4] == 2); // Should not update

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
