#include <TFT_eSPI.h>
#include <SPI.h>
#include "CommonLogic.h"

TFT_eSPI tft = TFT_eSPI();
#define BLACK 0x0000
#define WHITE 0xFFFF

#include "MAX6675.h"
MAX6675 thermoCouple(5);
uint32_t last_conv = 0;

const int analogInPin = 34;
float currentTemp = 0, currentLambda = 0;

struct Widget {
  int source; String label; int x, y, font, rate, mode; float minV, maxV; int p[5];
};

Widget layout[] = {
  {1, "Temp:", 10, 10, 2, 500, 1, 0, 1023, {0xFFFF}},
  {2, "Lam:", 170, 10, 2, 1000, 1, 0, 3.3, {0x07FF}},
  {1, "", 10, 50, 1, 500, 2, 20, 800, {0xFD20, 300, 8}},
  {1, "", 10, 80, 1, 500, 0x30, 30, 800, {0xF800, 140, 80}},
  {2, "", 170, 80, 1, 50, 4, 0, 3.3, {0x07E0, 60, 60, 40}}
};

float buffer0[140];

void readSensors() {
  if (millis() - last_conv >= 220) {
    thermoCouple.read(); currentTemp = thermoCouple.getTemperature(); last_conv = millis();
  }
  currentLambda = analogRead(analogInPin) * (3.3f / 4096.0f);
}

void drawWidget(Widget& w) {
  float val = (w.source == 1) ? currentTemp : currentLambda;
  if (w.mode == 1) {
    tft.fillRect(w.x + 60, w.y, 80, 20, BLACK);
    tft.setCursor(w.x, w.y); tft.setTextColor(w.p[0]); tft.setTextSize(w.font);
    tft.print(w.label); tft.print(val, 2);
  } else if (w.mode == 2) {
    tft.drawRect(w.x, w.y, w.p[1], w.p[2], w.p[0]);
    tft.fillRect(w.x+1, w.y+1, (int)fmap(val, w.minV, w.maxV, 0, w.p[1]-2), w.p[2]-2, w.p[0]);
  } else if (w.mode == 4) {
    tft.fillRect(w.x, w.y, w.p[1], w.p[2], BLACK);
    int cx = w.x + w.p[1]/2, cy = w.y + w.p[2]/2, r = w.p[3];
    float a = (fmap(val, w.minV, w.maxV, -90, 90) - 90) * PI / 180.0f;
    tft.drawCircle(cx, cy, r, w.p[0]);
    tft.drawLine(cx, cy, cx + (int)(r * cos(a)), cy + (int)(r * sin(a)), w.p[0]);
  } else if (w.mode == 0x30) {
    int h = w.p[2], v = (int)fmap(val, w.minV, w.maxV, 0, h);
    for(int j=0; j<139; j++) buffer0[j] = buffer0[j+1];
    buffer0[139] = v;
    tft.fillRect(w.x, w.y, 140, h, BLACK);
    for(int j=0; j<140; j++) tft.drawPixel(w.x + j, w.y + h - (int)buffer0[j], w.p[0]);
  }
}

void setup() {
  Serial.begin(115200); tft.init(); tft.setRotation(1); tft.fillScreen(BLACK);
  SPI.begin(); thermoCouple.begin();
}

void loop() {
  readSensors();
  static unsigned long lastUpd[5];
  for (int i = 0; i < 5; i++) {
    if (millis() - lastUpd[i] >= layout[i].rate) {
      lastUpd[i] = millis(); drawWidget(layout[i]);
    }
  }
}
