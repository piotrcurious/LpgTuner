#ifndef COMMON_LOGIC_H
#define COMMON_LOGIC_H

#ifdef Arduino_h
// Already included
#else
#include <cstdint>
#include <cmath>
#endif

// Constants for Mapping
#define RPM_BINS 10
#define LOAD_BINS 8

// Mapping helper functions
inline float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  if (std::abs(in_max - in_min) < 1e-6) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float rpmBinEdges[RPM_BINS + 1] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};
static float loadBinEdges[LOAD_BINS + 1] = {20, 40, 60, 80, 100, 120, 160, 200, 250};

inline int getRPMBin(float rpm) {
  for (int i = 0; i < RPM_BINS; i++) {
    if (rpm >= rpmBinEdges[i] && rpm < rpmBinEdges[i + 1]) {
      return i;
    }
  }
  return RPM_BINS - 1;
}

inline int getLoadBin(float load) {
  for (int i = 0; i < LOAD_BINS; i++) {
    if (load >= loadBinEdges[i] && load < loadBinEdges[i + 1]) {
      return i;
    }
  }
  return LOAD_BINS - 1;
}

inline uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

inline uint16_t getTempColor(float temp, float minTemp, float maxTemp) {
  if (temp < minTemp) temp = minTemp;
  if (temp > maxTemp) temp = maxTemp;
  float normalized = (temp - minTemp) / (maxTemp - minTemp);
  if (normalized < 0.2) return color565(0, 0, 255 * (1 - normalized / 0.2) + 128 * (normalized / 0.2));
  else if (normalized < 0.4) { float t = (normalized - 0.2) / 0.2; return color565(0, 255 * t, 128 + 127 * (1 - t)); }
  else if (normalized < 0.6) { float t = (normalized - 0.4) / 0.2; return color565(255 * t, 255, 0); }
  else if (normalized < 0.8) { float t = (normalized - 0.6) / 0.2; return color565(255, 255 * (1 - t), 0); }
  else { float t = (normalized - 0.8) / 0.2; return color565(255, 0, 0); }
}

inline uint16_t getLambdaColor(float lambda) {
  if (lambda < 0.8) lambda = 0.8;
  if (lambda > 1.2) lambda = 1.2;
  if (lambda < 1.0) { float t = (lambda - 0.8) / 0.2; return color565(255, 255 * t, 0); }
  else { float t = (lambda - 1.0) / 0.2; return color565(0, 255 * (1 - t), 255 * t); }
}

#define MAP_MIN_VOLTAGE 0.5
#define MAP_MAX_VOLTAGE 4.5
#define MAP_MIN_KPA 20.0
#define MAP_MAX_KPA 250.0

inline float voltageToKpa(float voltage) {
    float kpa = fmap(voltage, MAP_MIN_VOLTAGE, MAP_MAX_VOLTAGE, MAP_MIN_KPA, MAP_MAX_KPA);
    if (kpa < MAP_MIN_KPA) kpa = MAP_MIN_KPA;
    if (kpa > MAP_MAX_KPA) kpa = MAP_MAX_KPA;
    return kpa;
}

#endif
