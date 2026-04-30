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

#define MODE_DEC 1
#define MODE_BAR 2
#define MODE_GAUGE 4
#define MODE_HEATMAP_TEMP 5
#define MODE_HEATMAP_LAMBDA 6

Widget layouts[][6] = {
  {
    {1, "Temp:", 10, 10, 2, 500, MODE_DEC, 0, 1023, {0xFFFF, 120, 20}},
    {2, "Lam:", 170, 10, 2, 100, MODE_DEC, 0, 3.3, {0x07FF, 120, 20}},
    {3, "RPM:", 10, 40, 2, 100, MODE_DEC, 0, 10000, {0x07E0, 120, 20}},
    {4, "MAP:", 170, 40, 2, 100, MODE_DEC, 0, 250, {0xFFE0, 120, 20}},
    {1, "", 10, 80, 1, 500, MODE_BAR, 0, 1000, {0xFD20, 300, 10}},
    {3, "", 120, 110, 1, 100, MODE_GAUGE, 0, 10000, {0x07E0, 80, 80, 50}}
  },
  {
    {1, "", 10, 30, 1, 1000, MODE_HEATMAP_TEMP, 0, 850, {0xFFFF, 280, 160}},
    {0, "Temp Map", 80, 5, 2, 10000, MODE_DEC, 0, 0, {0xFFE0}},
    {2, "", 10, 30, 1, 1000, MODE_HEATMAP_LAMBDA, 0.5, 1.5, {0xFFFF, 280, 160}},
    {0,"",0,0,0,0,0,0,0,{0}},{0,"",0,0,0,0,0,0,0,{0}},{0,"",0,0,0,0,0,0,0,{0}}
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
  if (currentRPM > 500) {
    int r = getRPMBin(currentRPM), l = getLoadBin(currentMAP);
    if (currentTemp > 0 && currentLambda > 0) {
      int prev = sampleCount[r][l];
      updateMapSample(tempMap[r][l], currentTemp, sampleCount[r][l]);
      int dummy = prev;
      updateMapSample(lambdaMap[r][l], currentLambda, dummy);
    }
  }
}

void drawWidget(Widget& w) {
  float val = (w.source == 1) ? currentTemp : (w.source == 2) ? currentLambda : (w.source == 3) ? currentRPM : currentMAP;
  if (w.mode == MODE_DEC) {
    tft.fillRect(w.x + 60, w.y, 80, 20, BLACK);
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
  } else if (w.mode == MODE_HEATMAP_TEMP || w.mode == MODE_HEATMAP_LAMBDA) {
    int cw = w.p[1]/RPM_BINS, ch = w.p[2]/LOAD_BINS;
    bool isTemp = (w.mode == MODE_HEATMAP_TEMP);
    for(int r=0; r<RPM_BINS; r++) for(int l=0; l<LOAD_BINS; l++) {
      uint16_t c = sampleCount[r][l]>0 ? (isTemp ? getTempColor(tempMap[r][l], w.minV, w.maxV) : getLambdaColor(lambdaMap[r][l])) : 0x7BEF;
      tft.fillRect(w.x + r*cw, w.y + (LOAD_BINS-1-l)*ch, cw-1, ch-1, c);
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
  if (millis() - lastLC >= 10000) { lastLC = millis(); curL = (curL + 1) % 2; tft.fillScreen(BLACK); }
  static unsigned long lastUpd[2][6];
  for (int i = 0; i < 6; i++) {
    if (layouts[curL][i].label != "" || layouts[curL][i].mode != 0) {
      if (millis() - lastUpd[curL][i] >= layouts[curL][i].rate) {
        lastUpd[curL][i] = millis(); drawWidget(layouts[curL][i]);
      }
    }
  }
}
