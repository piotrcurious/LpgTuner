#include "mock_arduino.h"
#include <cassert>
#include <iostream>

// Include the logic from the ino file by defining a macro to skip Arduino-specific parts
// Or better, just copy the relevant functions here for unit testing.

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define RPM_BINS 10
#define LOAD_BINS 8
float rpmBinEdges[RPM_BINS + 1] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};
float loadBinEdges[LOAD_BINS + 1] = {20, 40, 60, 80, 100, 120, 160, 200, 250};

int getRPMBin(float rpm) {
  for (int i = 0; i < RPM_BINS; i++) {
    if (rpm >= rpmBinEdges[i] && rpm < rpmBinEdges[i + 1]) {
      return i;
    }
  }
  return RPM_BINS - 1;
}

int getLoadBin(float load) {
  for (int i = 0; i < LOAD_BINS; i++) {
    if (load >= loadBinEdges[i] && load < loadBinEdges[i + 1]) {
      return i;
    }
  }
  return LOAD_BINS - 1;
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
    assert(getRPMBin(12000) == 9);

    assert(getLoadBin(30) == 0);
    assert(getLoadBin(50) == 1);
    assert(getLoadBin(240) == 7);
    std::cout << "test_bins passed" << std::endl;
}

int main() {
    test_fmap();
    test_bins();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
