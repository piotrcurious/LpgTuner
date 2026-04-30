// TFT_eSPI library for ESP32
#include <TFT_eSPI.h>
#include <SPI.h>
#include "CommonLogic.h"

TFT_eSPI tft = TFT_eSPI();
#define BLACK 0x0000
#define WHITE 0xFFFF

#include "MAX6675.h"
const int selectPin = 5;
MAX6675 thermoCouple(selectPin);
uint32_t last_conversion_time = 0;
#define MAX6675_CONVERSION_RATE 200

const int analogInPin = 34;
const int mapSensorPin = 35;
const int rpmInputPin = 32;
#define MAX_ANALOG_VOLTAGE_A0 3.3
#define ADC_RESOLUTION_A0 4096

float currentTemp = 0;
float currentLambda = 0;
float currentMAP = 0;
float currentRPM = 0;
volatile unsigned long rpmPulseCount = 0;
unsigned long lastRPMCalcTime = 0;
#define RPM_CALC_INTERVAL 100
#define PULSES_PER_REV 2

float tempMap[RPM_BINS][LOAD_BINS];
float lambdaMap[RPM_BINS][LOAD_BINS];
int sampleCount[RPM_BINS][LOAD_BINS];

struct WidgetElement {
  String name; int source; String label; int x; int y; int font; int rate; int mode; float value_min; float value_max; int params[10];
};

#define MODE_DEC 1
#define MODE_BAR 2

WidgetElement layouts[][6] = {
  {
    {"Temp", 1, "T:", 10, 10, 2, 500, MODE_DEC, 0, 1000, {0, 0, 0xFD20, 120, 20}},
    {"Lambda", 2, "L:", 170, 10, 2, 100, MODE_DEC, 0, 2, {0, 0, 0x07FF, 120, 20}},
    {"RPM", 3, "RPM:", 10, 40, 2, 100, MODE_DEC, 0, 10000, {0, 0, 0x07E0, 120, 20}},
    {"MAP", 4, "MAP:", 170, 40, 2, 100, MODE_DEC, 0, 250, {0, 0, 0xFFE0, 120, 20}},
    {"TBar", 1, "", 10, 80, 1, 500, MODE_BAR, 0, 1000, {0, 0, 0xFD20, 300, 10}},
    {"RGauge", 3, "", 120, 110, 1, 100, 4, 0, 10000, {0, 0, 0x07E0, 80, 80, 50}}
  }
};

#define NUM_LAYOUTS 1
#define NUM_WIDGETS 6
unsigned long lastUpdate[NUM_WIDGETS];

void IRAM_ATTR rpmPulseISR() { rpmPulseCount++; }

void readSensors() {
  if (millis() - last_conversion_time >= MAX6675_CONVERSION_RATE) {
    thermoCouple.read(); currentTemp = thermoCouple.getTemperature(); last_conversion_time = millis();
  }
  currentLambda = (analogRead(analogInPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0));
  currentMAP = voltageToKpa(analogRead(mapSensorPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0));
  if (millis() - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
    noInterrupts(); unsigned long pulses = rpmPulseCount; rpmPulseCount = 0; interrupts();
    currentRPM = (pulses / PULSES_PER_REV) * (60.0 / (RPM_CALC_INTERVAL / 1000.0));
    lastRPMCalcTime = millis();
  }
}

void updateHeatMaps() {
  if (currentRPM > 500) {
    int r = getRPMBin(currentRPM), l = getLoadBin(currentMAP);
    if (currentTemp > 0 && currentLambda > 0) {
      if (sampleCount[r][l] == 0) { tempMap[r][l] = currentTemp; lambdaMap[r][l] = currentLambda; sampleCount[r][l] = 1; }
      else { tempMap[r][l] = tempMap[r][l] * 0.9 + currentTemp * 0.1; lambdaMap[r][l] = lambdaMap[r][l] * 0.9 + currentLambda * 0.1; sampleCount[r][l]++; }
    }
  }
}

void setup() {
  Serial.begin(115200); tft.init(); tft.setRotation(1); tft.fillScreen(BLACK);
  SPI.begin(); thermoCouple.begin();
  pinMode(rpmInputPin, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(rpmInputPin), rpmPulseISR, FALLING);
}

void loop() {
  readSensors(); updateHeatMaps();
  WidgetElement* layout = layouts[0];
  for (int i = 0; i < NUM_WIDGETS; i++) {
    if (millis() - lastUpdate[i] >= layout[i].rate) {
      lastUpdate[i] = millis();
      float val = (layout[i].source == 1) ? currentTemp : (layout[i].source == 2) ? currentLambda : (layout[i].source == 3) ? currentRPM : currentMAP;
      tft.fillRect(layout[i].x, layout[i].y, 150, 20, BLACK);
      tft.setCursor(layout[i].x, layout[i].y); tft.setTextColor(layout[i].params[2]); tft.print(layout[i].label); tft.print(val, 1);
    }
  }
}
