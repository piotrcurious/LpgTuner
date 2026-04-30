#include <TFT_eSPI.h>
#include <SPI.h>
#include "CommonLogic.h"

TFT_eSPI tft = TFT_eSPI();
#define BLACK 0x0000
#define WHITE 0xFFFF

#include "MAX6675.h"
MAX6675 thermoCouple(5);
uint32_t last_conv = 0;

const int analogInPin = 34, mapPin = 35, rpmPin = 32;
float currentTemp = 0, currentLambda = 0, currentMAP = 0, currentRPM = 0;
volatile unsigned long rpmPulses = 0;
unsigned long lastRPMCalc = 0;

float tempMap[RPM_BINS][LOAD_BINS], lambdaMap[RPM_BINS][LOAD_BINS];
int sampleCount[RPM_BINS][LOAD_BINS];

struct Widget {
  int source; String label; int x, y, font, rate, mode; float minV, maxV; int p[5];
};

Widget layouts[][6] = {
  { // Layout 1: Dash
    {1, "Temp:", 10, 10, 2, 500, MODE_DEC, 0, 1000, {0xFFFF}},
    {2, "Lam:", 170, 10, 2, 100, MODE_DEC, 0, 3.3, {0x07FF}},
    {3, "RPM:", 10, 40, 2, 100, MODE_DEC, 0, 10000, {0x07E0}},
    {4, "MAP:", 170, 40, 2, 100, MODE_DEC, 0, 250, {0xFFE0}},
    {1, "", 10, 80, 1, 500, MODE_BAR, 0, 1000, {0xFD20, 300, 10}},
    {3, "", 120, 110, 1, 100, MODE_GAUGE, 0, 10000, {0x07E0, 80, 80, 50}}
  },
  { // Layout 2: Maps
    {1, "", 10, 30, 1, 1000, MODE_HEATMAP_TEMP, 0, 850, {0xFFFF, 280, 160}},
    {0, "Temp Map", 80, 5, 2, 10000, MODE_DEC, 0, 0, {0xFFE0}},
    {2, "", 10, 30, 1, 1000, MODE_HEATMAP_LAMBDA, 0.5, 1.5, {0xFFFF, 280, 160}},
    {0, "", 0, 0, 0, 0, 0, 0, 0, {0}}, {0, "", 0, 0, 0, 0, 0, 0, 0, {0}}, {0, "", 0, 0, 0, 0, 0, 0, 0, {0}}
  },
  { // Layout 3: Combined
    {1, "", 10, 30, 1, 1000, MODE_HEATMAP_COMBINED, 0, 850, {0xFFFF, 280, 160}},
    {0, "Combined Map", 80, 5, 2, 10000, MODE_DEC, 0, 0, {0x07FF}},
    {1, "T:", 10, 215, 1, 500, MODE_DEC, 0, 1000, {0xFD20}},
    {2, "L:", 90, 215, 1, 100, MODE_DEC, 0, 2, {0x07FF}},
    {3, "RPM:", 170, 215, 1, 100, MODE_DEC, 0, 10000, {0x07E0}},
    {4, "MAP:", 250, 215, 1, 100, MODE_DEC, 0, 250, {0xFFE0}}
  }
};

void IRAM_ATTR rpmISR() { rpmPulses++; }

void readSensors() {
  if (millis() - last_conv >= 220) {
    thermoCouple.read(); currentTemp = thermoCouple.getTemperature(); last_conv = millis();
  }
  currentLambda = analogRead(analogInPin) * (3.3f / 4096.0f);
  currentMAP = voltageToKpa(analogRead(mapPin) * (3.3f / 4096.0f));
  if (millis() - lastRPMCalc >= 100) {
    unsigned long now = millis();
    noInterrupts(); unsigned long p = rpmPulses; rpmPulses = 0; interrupts();
    currentRPM = (p / 2.0f) * (60.0f / ((now - lastRPMCalc) / 1000.0f));
    lastRPMCalc = now;
  }
}

void updateHeatMaps() {
  if (currentRPM > 500 && currentTemp > 0 && currentLambda > 0) {
    int r = getRPMBin(currentRPM), l = getLoadBin(currentMAP);
    int prev = sampleCount[r][l];
    updateMapSample(tempMap[r][l], currentTemp, sampleCount[r][l]);
    int d = prev;
    updateMapSample(lambdaMap[r][l], currentLambda, d);
  }
}

void drawWidget(Widget& w, int idx) {
  float val = (w.source == 1) ? currentTemp : (w.source == 2) ? currentLambda : (w.source == 3) ? currentRPM : currentMAP;
  if (w.mode == MODE_DEC) {
    tft.fillRect(w.x + 55, w.y, 65, 20, BLACK);
    tft.setCursor(w.x, w.y); tft.setTextColor(w.p[0]); tft.setTextSize(w.font);
    tft.print(w.label); tft.print(val, (w.source == 3) ? 0 : (w.source == 2) ? 2 : 1);
  } else if (w.mode == MODE_BAR) {
    tft.drawRect(w.x, w.y, w.p[1], w.p[2], w.p[0]);
    tft.fillRect(w.x+1, w.y+1, (int)fmap(val, w.minV, w.maxV, 0, w.p[1]-2), w.p[2]-2, w.p[0]);
  } else if (w.mode == MODE_GAUGE) {
    int cx = w.x + w.p[1]/2, cy = w.y + w.p[2]/2, r = w.p[3];
    float a = (fmap(val, w.minV, w.maxV, -90, 90) - 90) * PI / 180.0f;
    tft.drawCircle(cx, cy, r, w.p[0]);
    tft.drawLine(cx, cy, cx + (int)(r * cos(a)), cy + (int)(r * sin(a)), w.p[0]);
  } else if (w.mode >= MODE_HEATMAP_TEMP && w.mode <= MODE_HEATMAP_COMBINED) {
    int cw = w.p[1]/RPM_BINS, ch = w.p[2]/LOAD_BINS;
    for(int r=0; r<RPM_BINS; r++) for(int l=0; l<LOAD_BINS; l++) {
      int x = w.x + r*cw, y = w.y + (LOAD_BINS-1-l)*ch;
      if (sampleCount[r][l] == 0) { tft.fillRect(x, y, cw-1, ch-1, 0x7BEF); continue; }
      uint16_t bg = (w.mode == MODE_HEATMAP_LAMBDA) ? getLambdaColor(lambdaMap[r][l]) : getTempColor(tempMap[r][l], w.minV, w.maxV);
      tft.fillRect(x, y, cw-1, ch-1, bg);
      if (w.mode == MODE_HEATMAP_COMBINED) {
        LambdaZone z = getLambdaZone(lambdaMap[r][l]);
        int mx = x + cw/2 - 2, my = y + ch/2 - 2;
        if (z == LAMBDA_LEAN) tft.drawRect(mx, my, 4, 4, WHITE);
        else if (z == LAMBDA_RICH) tft.fillCircle(mx+2, my+2, 2, WHITE);
        else tft.fillRect(mx, my, 2, 4, WHITE);
      }
      if (r == getRPMBin(currentRPM) && l == getLoadBin(currentMAP) && currentRPM > 500)
          tft.drawRect(x, y, cw-1, ch-1, WHITE);
    }
  }
}

void setup() {
  Serial.begin(115200); tft.init(); tft.setRotation(1); tft.fillScreen(BLACK);
  SPI.begin(); thermoCouple.begin();
  pinMode(rpmPin, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(rpmPin), rpmISR, FALLING);
}

void loop() {
  readSensors(); updateHeatMaps();
  static unsigned long lastLC = 0; static int curL = 0;
  if (millis() - lastLC >= 8000) { lastLC = millis(); curL = (curL + 1) % 3; tft.fillScreen(BLACK); }
  static unsigned long lastUpd[3][6];
  for (int i = 0; i < 6; i++) {
    Widget& w = layouts[curL][i];
    if (w.label != "" || w.mode != 0) {
      if (millis() - lastUpd[curL][i] >= w.rate) {
        lastUpd[curL][i] = millis(); drawWidget(w, i);
      }
    }
  }
}
