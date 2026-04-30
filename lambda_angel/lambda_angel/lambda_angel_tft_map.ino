// TFT_eSPI library for ESP32
#include <TFT_eSPI.h>
#include <SPI.h>
#include "CommonLogic.h"

TFT_eSPI tft = TFT_eSPI();
#define BLACK 0x0000

#include "MAX6675.h"
MAX6675 thermoCouple(5);
uint32_t last_conversion_time = 0; 
#define MAX6675_CONVERSION_RATE 200

const int analogInPin = 34;
#define MAX_ANALOG_VOLTAGE_A0 3.3
#define ADC_RESOLUTION_A0 4096

float currentTemp = 0;
float currentLambda = 0;

struct WidgetElement {
  String name; int source; String label; int x; int y; int font; int rate; int mode; float value_min; float value_max; int params[10];
};

WidgetElement layouts[][5] = {
  {
    {"Temp", 1, "T:", 10, 10, 2, 500, 1, 0, 1023, {0, 0, 0xFFFF, 120, 20}},
    {"Lambda", 2, "L:", 170, 10, 2, 1000, 1, 0, 3.3, {0, 0, 0x07FF, 120, 20}},
    {"TBar", 1, "", 10, 50, 1, 500, 0x2, 20, 800, {0, 0, 0xFD20, 300, 8}},
    {"TGraph", 1, "", 10, 80, 1, 500, 0x30, 30, 800, {0, 0, 0xF800, 140, 80}},
    {"LGauge", 2, "", 170, 80, 1, 50, 4, 0, 3.3, {0, 0, 0x07E0, 60, 60, 40}}
  }
};

unsigned long lastUpdate[5];

void readSensors() {
  if (millis() - last_conversion_time >= MAX6675_CONVERSION_RATE) {
    thermoCouple.read(); currentTemp = thermoCouple.getTemperature(); last_conversion_time = millis();
  }
  currentLambda = analogRead(analogInPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0);
}

void setup() {
  Serial.begin(115200); tft.init(); tft.setRotation(1); tft.fillScreen(BLACK);
  SPI.begin(); thermoCouple.begin();
}

void loop() {
  readSensors();
  for (int i = 0; i < 5; i++) {
    WidgetElement w = layouts[0][i];
    if (millis() - lastUpdate[i] >= w.rate) {
      lastUpdate[i] = millis();
      float val = (w.source == 1) ? currentTemp : currentLambda;
      tft.fillRect(w.x, w.y, w.params[3], w.params[4]+1, BLACK);
      tft.setCursor(w.x, w.y); tft.setTextColor(w.params[2]); tft.setTextSize(w.font);
      if (w.label != "") { tft.print(w.label); tft.print(val, 2); }
    }
  }
}
