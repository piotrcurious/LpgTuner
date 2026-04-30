#ifndef COMMON_LOGIC_H
#define COMMON_LOGIC_H

#ifdef Arduino_h
// Already included or in mock environment
#else
#include <cstdint>
#include <cmath>
#include <algorithm>
#endif

// Constants for Mapping
#define RPM_BINS 10
#define LOAD_BINS 8

// Widget mode definitions
#define MODE_HEX 0
#define MODE_DEC 1
#define MODE_BAR 2
#define MODE_BAR_FADE 0x21
#define MODE_GRAPH_0 0x30
#define MODE_GRAPH_1 0x31
#define MODE_GAUGE 4
#define MODE_HEATMAP_TEMP 5
#define MODE_HEATMAP_LAMBDA 6
#define MODE_HEATMAP_COMBINED 7

enum LambdaZone { LAMBDA_LEAN, LAMBDA_STOICH, LAMBDA_RICH };

// Mapping helper functions
inline float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  if (std::abs(in_max - in_min) < 1e-6) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const float rpmBinEdges[RPM_BINS + 1] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};
const float loadBinEdges[LOAD_BINS + 1] = {20, 40, 60, 80, 100, 120, 160, 200, 250};

inline int getRPMBin(float rpm) {
  if (rpm < 0) return 0;
  for (int i = 0; i < RPM_BINS; i++) {
    if (rpm >= rpmBinEdges[i] && rpm < rpmBinEdges[i + 1]) return i;
  }
  return RPM_BINS - 1;
}

inline int getLoadBin(float load) {
  if (load < loadBinEdges[0]) return 0;
  for (int i = 0; i < LOAD_BINS; i++) {
    if (load >= loadBinEdges[i] && load < loadBinEdges[i + 1]) return i;
  }
  return LOAD_BINS - 1;
}

// Filter logic
inline float lowPass(float current, float newValue, float alpha = 0.1f) {
    return current * (1.0f - alpha) + newValue * alpha;
}

// Unified Map Update Logic
inline void updateMapSample(float& mapValue, float newValue, int& count, float alpha = 0.1f) {
  if (count == 0) {
    mapValue = newValue;
    count = 1;
  } else {
    mapValue = lowPass(mapValue, newValue, alpha);
    count++;
  }
}

inline LambdaZone getLambdaZone(float lambda) {
    if (lambda > 1.05f) return LAMBDA_LEAN;
    if (lambda < 0.95f) return LAMBDA_RICH;
    return LAMBDA_STOICH;
}

// Color conversion (RGB565)
inline uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

inline uint16_t getTempColor(float temp, float minTemp, float maxTemp) {
  float range = maxTemp - minTemp;
  if (range < 1e-6) return 0xFFFF;
  float normalized = (std::max(std::min(temp, maxTemp), minTemp) - minTemp) / range;

  if (normalized < 0.25f) return color565(0, (uint8_t)(normalized * 1020), 255);
  if (normalized < 0.50f) return color565(0, 255, (uint8_t)(255 - (normalized - 0.25f) * 1020));
  if (normalized < 0.75f) return color565((uint8_t)((normalized - 0.50f) * 1020), 255, 0);
  return color565(255, (uint8_t)(255 - (normalized - 0.75f) * 1020), 0);
}

inline uint16_t getLambdaColor(float lambda) {
  if (lambda < 0.8f) return color565(255, 0, 0);   // Very Rich
  if (lambda > 1.2f) return color565(0, 0, 255);   // Very Lean
  if (lambda < 1.0f) return color565(255, (uint8_t)((lambda - 0.8f) * 1275), 0);
  return color565((uint8_t)(255 - (lambda - 1.0f) * 1275), 255, (uint8_t)((lambda - 1.0f) * 1275));
}

#define MAP_MIN_VOLTAGE 0.5f
#define MAP_MAX_VOLTAGE 4.5f
#define MAP_MIN_KPA 20.0f
#define MAP_MAX_KPA 250.0f

inline float voltageToKpa(float voltage) {
    float kpa = fmap(voltage, MAP_MIN_VOLTAGE, MAP_MAX_VOLTAGE, MAP_MIN_KPA, MAP_MAX_KPA);
    return std::max(std::min(kpa, MAP_MAX_KPA), MAP_MIN_KPA);
}

#endif
