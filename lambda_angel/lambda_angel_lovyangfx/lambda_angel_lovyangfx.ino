#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "CommonLogic.h"

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read  = 16000000;
      cfg.pin_sclk = 18;
      cfg.pin_mosi = 23;
      cfg.pin_miso = 19;
      cfg.pin_dc   = 2;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 15;
      cfg.pin_rst          = 4;
      cfg.panel_width      = 240;
      cfg.panel_height     = 320;
      cfg.bus_shared       = true;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;

#include "MAX6675.h"
const int selectPin = 5;
MAX6675 thermoCouple(selectPin);
uint32_t last_conversion_time = 0;
#define MAX6675_CONVERSION_RATE 200

const int analogInPin = 34;
const int mapSensorPin = 35;
const int rpmInputPin = 32;

float currentTemp = 0, currentLambda = 0, currentMAP = 0, currentRPM = 0;
volatile unsigned long rpmPulseCount = 0;
unsigned long lastRPMCalcTime = 0;
#define RPM_CALC_INTERVAL 100
#define PULSES_PER_REV 2

float tempMap[RPM_BINS][LOAD_BINS], lambdaMap[RPM_BINS][LOAD_BINS];
int sampleCount[RPM_BINS][LOAD_BINS];

struct WidgetElement {
  String name; int source; String label; int x; int y; int font; int rate; int mode; float value_min; float value_max; int params[10];
};

#define MODE_DEC 1
#define MODE_BAR 2
#define MODE_GAUGE 4
#define MODE_HEATMAP_TEMP 5
#define MODE_HEATMAP_LAMBDA 6

WidgetElement layouts[][6] = {
  {
    {"Temp", 1, "T:", 10, 10, 2, 500, MODE_DEC, 0, 1000, {0, 0, 0xFD20, 120, 20}},
    {"Lambda", 2, "L:", 170, 10, 2, 100, MODE_DEC, 0, 2, {0, 0, 0x07FF, 120, 20}},
    {"RPM", 3, "RPM:", 10, 40, 2, 100, MODE_DEC, 0, 10000, {0, 0, 0x07E0, 120, 20}},
    {"MAP", 4, "MAP:", 170, 40, 2, 100, MODE_DEC, 0, 250, {0, 0, 0xFFE0, 120, 20}},
    {"TBar", 1, "", 10, 80, 1, 500, MODE_BAR, 0, 1000, {0, 0, 0xFD20, 300, 10}},
    {"RGauge", 3, "", 120, 110, 1, 100, MODE_GAUGE, 0, 10000, {0, 0, 0x07E0, 80, 80, 50}}
  },
  {
    {"tempmap", 1, "", 10, 30, 1, 1000, MODE_HEATMAP_TEMP, 0, 850, {0, 0, 0xFFFF, 280, 160}},
    {"title", 0, "Engine Maps", 80, 5, 2, 10000, MODE_DEC, 0, 0, {0, 0, 0xFFE0}},
    {"",0,"",0,0,0,0,0,0,0,{0}},{"",0,"",0,0,0,0,0,0,0,{0}},{"",0,"",0,0,0,0,0,0,0,{0}},{"",0,"",0,0,0,0,0,0,0,{0}}
  }
};

void IRAM_ATTR rpmPulseISR() { rpmPulseCount++; }

void readSensors() {
  if (millis() - last_conversion_time >= MAX6675_CONVERSION_RATE) {
    thermoCouple.read(); currentTemp = thermoCouple.getTemperature(); last_conversion_time = millis();
  }
  currentLambda = (analogRead(analogInPin) * (3.3 / 4096.0)) / 3.3;
  currentMAP = voltageToKpa(analogRead(mapSensorPin) * (3.3 / 4096.0));
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
      int prev = sampleCount[r][l]; updateMapSample(tempMap[r][l], currentTemp, sampleCount[r][l]);
      int d = prev; updateMapSample(lambdaMap[r][l], currentLambda, d);
    }
  }
}

void setup() {
  Serial.begin(115200); tft.init(); tft.setRotation(1); tft.fillScreen(0);
  SPI.begin(); thermoCouple.begin();
  pinMode(rpmInputPin, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(rpmInputPin), rpmPulseISR, FALLING);
}

void loop() {
  readSensors(); updateHeatMaps();
  static unsigned long lastLC = 0; static int curL = 0;
  if (millis() - lastLC >= 10000) { lastLC = millis(); curL = (curL + 1) % 2; tft.fillScreen(0); }
  WidgetElement* layout = layouts[curL];
  for (int i = 0; i < 6; i++) {
    static unsigned long lastUpd[6];
    if (layout[i].name != "" && millis() - lastUpd[i] >= layout[i].rate) {
      lastUpd[i] = millis();
      float val = (layout[i].source == 1) ? currentTemp : (layout[i].source == 2) ? currentLambda : (layout[i].source == 3) ? currentRPM : currentMAP;
      if (layout[i].mode == MODE_HEATMAP_TEMP) {
        int cw = layout[i].params[3]/RPM_BINS, ch = layout[i].params[4]/LOAD_BINS;
        for(int r=0; r<RPM_BINS; r++) for(int l=0; l<LOAD_BINS; l++) {
          uint16_t c = sampleCount[r][l]>0 ? getTempColor(tempMap[r][l], layout[i].value_min, layout[i].value_max) : 0x7BEF;
          tft.fillRect(layout[i].x + r*cw, layout[i].y + (LOAD_BINS-1-l)*ch, cw-1, ch-1, c);
        }
      } else {
        tft.fillRect(layout[i].x, layout[i].y, 150, 20, 0); tft.setCursor(layout[i].x, layout[i].y);
        tft.setTextColor(layout[i].params[2]); tft.setTextSize(layout[i].font);
        tft.print(layout[i].label); tft.print(val, 1);
        if (layout[i].mode == MODE_BAR) tft.fillRect(layout[i].x, layout[i].y+15, fmap(val, layout[i].value_min, layout[i].value_max, 0, layout[i].params[3]), layout[i].params[4], layout[i].params[2]);
      }
    }
  }
}
