#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "CommonLogic.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 4, 0, 15);

#include "MAX6675.h"
MAX6675 thermoCouple(2, &SPI);
uint32_t last_conversion_time = 0;
#define MAX6675_CONVERSION_RATE 200

const int analogInPin = A0;
#define MAX_ANALOG_VOLTAGE_A0 3.3
#define ADC_RESOLUTION_A0 1024

float currentTemp = 0;
float currentLambda = 0;

struct WidgetElement {
  String name; int source; String label; int x; int y; int font; int rate; int mode; float value_min; float value_max; int params[10];
};

WidgetElement layouts[][5] = {
  {
    {"Temp", 1, "T:", 0, 0, 1, 500, 1, 0, 1023, {0, 1023, WHITE, 64, 8}},
    {"Lambda", 2, "l:", 64, 0, 1, 1000, 1, 0, 5.0, {0, 1023, WHITE, 64, 8}},
    {"TBar", 1, "", 0, 8, 1, 500, 0x2, 20, 800, {20, 800, WHITE, 128, 4}},
    {"TGraph", 1, "", 0, 12, 1, 500, 0x30, 30, 800, {30, 800, WHITE, 64, 32}},
    {"LGauge", 2, "", 64, 12, 1, 50, 4, 0, 3.3, {0, 3.3, WHITE, 20, 20, 14}}
  }
};

unsigned long lastUpdate[5];
float buffer0[64];

void readSensors() {
  if (millis() - last_conversion_time >= MAX6675_CONVERSION_RATE) {
    thermoCouple.read(); currentTemp = thermoCouple.getTemperature(); last_conversion_time = millis();
  }
  currentLambda = analogRead(analogInPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0);
}

void setup() {
  Serial.begin(115200); display.begin(SSD1306_SWITCHCAPVCC); SPI.begin(); thermoCouple.begin();
  display.clearDisplay(); display.display();
}

void loop() {
  readSensors();
  bool changed = false;
  for (int i = 0; i < 5; i++) {
    WidgetElement w = layouts[0][i];
    if (millis() - lastUpdate[i] >= w.rate) {
      lastUpdate[i] = millis(); changed = true;
      float val = (w.source == 1) ? currentTemp : currentLambda;
      display.fillRect(w.x, w.y, w.params[3], w.params[4]+1, BLACK);
      display.setCursor(w.x, w.y); display.setTextColor(WHITE); display.setTextSize(w.font);
      if (w.label != "") { display.print(w.label); display.print(val, 2); }
      if (w.mode == 0x2) display.fillRect(w.x, w.y, fmap(val, w.value_min, w.value_max, 0, w.params[3]), w.params[4], WHITE);
      else if (w.mode == 4) {
        float a = fmap(val, w.value_min, w.value_max, -90, 90) * PI / 180.0;
        int cx = w.x + w.params[3]/2, cy = w.y + w.params[4]/2, r = w.params[5];
        display.drawLine(cx, cy, cx + r*cos(a), cy + r*sin(a), WHITE);
      } else if (w.mode == 0x30) {
        int h = w.params[4], v = fmap(val, w.value_min, w.value_max, 0, h);
        for(int j=0; j<63; j++) buffer0[j] = buffer0[j+1]; buffer0[63] = v;
        for(int j=0; j<64; j++) display.drawPixel(w.x + j, w.y + h - buffer0[j], WHITE);
      }
    }
  }
  if (changed) display.display();
}
