#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "CommonLogic.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 4, 0, 15);

#include "MAX6675.h"
MAX6675 thermoCouple(2, &SPI);
uint32_t last_conv = 0;

const int analogInPin = A0;
float currentTemp = 0, currentLambda = 0;

struct Widget {
  int source; String label; int x, y, font, rate, mode; float minV, maxV; int p[5];
};

Widget layout[] = {
  {1, "T:", 0, 0, 1, 500, 1, 0, 1023, {WHITE}},
  {2, "L:", 64, 0, 1, 1000, 1, 0, 5.0, {WHITE}},
  {1, "", 0, 8, 1, 500, 2, 20, 800, {WHITE, 128, 4}},
  {1, "", 0, 12, 1, 500, 0x30, 30, 800, {WHITE, 64, 32}},
  {2, "", 64, 12, 1, 50, 4, 0, 3.3, {WHITE, 20, 20, 14}}
};

float buffer0[64];

void readSensors() {
  if (millis() - last_conv >= 220) {
    thermoCouple.read(); currentTemp = thermoCouple.getTemperature(); last_conv = millis();
  }
  currentLambda = analogRead(analogInPin) * (3.3f / 1024.0f);
}

void drawWidget(Widget& w) {
  float val = (w.source == 1) ? currentTemp : currentLambda;
  if (w.mode == 1) {
    display.fillRect(w.x + 15, w.y, 45, 8, BLACK);
    display.setCursor(w.x, w.y); display.setTextColor(WHITE); display.setTextSize(w.font);
    display.print(w.label); display.print(val, 1);
  } else if (w.mode == 2) {
    display.drawRect(w.x, w.y, w.p[1], w.p[2], WHITE);
    display.fillRect(w.x+1, w.y+1, (int)fmap(val, w.minV, w.maxV, 0, w.p[1]-2), w.p[2]-2, WHITE);
  } else if (w.mode == 4) {
    display.fillRect(w.x, w.y, w.p[1], w.p[2], BLACK);
    int cx = w.x + w.p[1]/2, cy = w.y + w.p[2]/2, r = w.p[3];
    float a = (fmap(val, w.minV, w.maxV, -90, 90) - 90) * PI / 180.0f;
    display.drawCircle(cx, cy, r, WHITE);
    display.drawLine(cx, cy, cx + (int)(r * cos(a)), cy + (int)(r * sin(a)), WHITE);
  } else if (w.mode == 0x30) {
    int h = w.p[2], v = (int)fmap(val, w.minV, w.maxV, 0, h);
    for(int j=0; j<63; j++) buffer0[j] = buffer0[j+1];
    buffer0[63] = v;
    display.fillRect(w.x, w.y, 64, h, BLACK);
    for(int j=0; j<64; j++) display.drawPixel(w.x + j, w.y + h - (int)buffer0[j], WHITE);
  }
}

void setup() {
  Serial.begin(115200); display.begin(SSD1306_SWITCHCAPVCC); SPI.begin(); thermoCouple.begin();
  display.clearDisplay(); display.display();
}

void loop() {
  readSensors();
  static unsigned long lastUpd[5]; bool changed = false;
  for (int i = 0; i < 5; i++) {
    if (millis() - lastUpd[i] >= layout[i].rate) {
      lastUpd[i] = millis(); drawWidget(layout[i]); changed = true;
    }
  }
  if (changed) display.display();
}
