#define LGFX_USE_V1
#include <LovyanGFX.hpp>

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

#define MAP_MIN_VOLTAGE 0.5
#define MAP_MAX_VOLTAGE 4.5
#define MAP_MIN_KPA 20.0
#define MAP_MAX_KPA 250.0

#define RPM_BINS 10
#define LOAD_BINS 8
float rpmBinEdges[RPM_BINS + 1] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};
float loadBinEdges[LOAD_BINS + 1] = {20, 40, 60, 80, 100, 120, 160, 200, 250};
float tempMap[RPM_BINS][LOAD_BINS];
float lambdaMap[RPM_BINS][LOAD_BINS];
int sampleCount[RPM_BINS][LOAD_BINS];

struct WidgetElement {
  String name;
  int source;
  String label;
  int x;
  int y;
  int font;
  int rate;
  int mode;
  float value_min;
  float value_max;
  int params[10];
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
    {"title", 0, "Temp Map", 80, 5, 2, 10000, MODE_DEC, 0, 0, {0, 0, 0xFFE0}},
    {"", 0, "", 0, 0, 0, 0, 0, 0, 0, {0}}, {"", 0, "", 0, 0, 0, 0, 0, 0, 0, {0}}, {"", 0, "", 0, 0, 0, 0, 0, 0, 0, {0}}, {"", 0, "", 0, 0, 0, 0, 0, 0, 0, {0}}
  }
};

#define NUM_LAYOUTS (sizeof(layouts) / sizeof(layouts[0]))
#define NUM_WIDGETS (sizeof(layouts[0]) / sizeof(layouts[0][0]))
unsigned long lastUpdate[NUM_WIDGETS];
unsigned long lastLayoutChange = 0;
unsigned long layoutChangeInterval = 10000;
int currentLayout = 0;

void IRAM_ATTR rpmPulseISR() { rpmPulseCount++; }

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readSensors() {
  if (millis() - last_conversion_time >= MAX6675_CONVERSION_RATE) {
    thermoCouple.read();
    currentTemp = thermoCouple.getTemperature();
    last_conversion_time = millis();
  }
  currentLambda = (analogRead(analogInPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0)) / 3.3;
  float mapV = analogRead(mapSensorPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0);
  currentMAP = fmap(mapV, MAP_MIN_VOLTAGE, MAP_MAX_VOLTAGE, MAP_MIN_KPA, MAP_MAX_KPA);
  if (millis() - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
    noInterrupts();
    unsigned long pulses = rpmPulseCount;
    rpmPulseCount = 0;
    interrupts();
    float timeSeconds = (millis() - lastRPMCalcTime) / 1000.0;
    currentRPM = (pulses / PULSES_PER_REV) * (60.0 / timeSeconds);
    lastRPMCalcTime = millis();
  }
}

void updateHeatMaps() {
  if (currentRPM > 500) {
    int r = 0; while(r < RPM_BINS-1 && currentRPM > rpmBinEdges[r+1]) r++;
    int l = 0; while(l < LOAD_BINS-1 && currentMAP > loadBinEdges[l+1]) l++;
    if (currentTemp > 0 && currentLambda > 0) {
      if (sampleCount[r][l] == 0) {
        tempMap[r][l] = currentTemp; lambdaMap[r][l] = currentLambda; sampleCount[r][l] = 1;
      } else {
        tempMap[r][l] = tempMap[r][l] * 0.9 + currentTemp * 0.1;
        lambdaMap[r][l] = lambdaMap[r][l] * 0.9 + currentLambda * 0.1;
        sampleCount[r][l]++;
      }
    }
  }
}

uint16_t getTempColor(float temp, float minTemp, float maxTemp) {
  float normalized = (fmin(fmax(temp, minTemp), maxTemp) - minTemp) / (maxTemp - minTemp);
  if (normalized < 0.5) return tft.color565(normalized * 510, 255, 255 * (1 - normalized * 2));
  return tft.color565(255, 255 * (1 - (normalized - 0.5) * 2), 0);
}

void setup() {
  Serial.begin(115200);
  tft.init(); tft.setRotation(1); tft.fillScreen(0);
  SPI.begin(); thermoCouple.begin();
  pinMode(rpmInputPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmInputPin), rpmPulseISR, FALLING);
}

void loop() {
  readSensors();
  updateHeatMaps();
  if (millis() - lastLayoutChange >= layoutChangeInterval) {
    lastLayoutChange = millis();
    currentLayout = (currentLayout + 1) % NUM_LAYOUTS;
    tft.fillScreen(0);
    for(int i=0; i<NUM_WIDGETS; i++) lastUpdate[i] = 0;
  }
  WidgetElement* layout = layouts[currentLayout];
  for (int i = 0; i < NUM_WIDGETS; i++) {
    if (layout[i].name != "" && millis() - lastUpdate[i] >= layout[i].rate) {
      lastUpdate[i] = millis();
      float val = (layout[i].source == 1) ? currentTemp : (layout[i].source == 2) ? currentLambda : (layout[i].source == 3) ? currentRPM : currentMAP;
      if (layout[i].mode == MODE_HEATMAP_TEMP) {
        int cw = layout[i].params[3]/RPM_BINS, ch = layout[i].params[4]/LOAD_BINS;
        for(int r=0; r<RPM_BINS; r++) for(int l=0; l<LOAD_BINS; l++) {
          uint16_t c = sampleCount[r][l]>0 ? getTempColor(tempMap[r][l], layout[i].value_min, layout[i].value_max) : 0x7BEF;
          tft.fillRect(layout[i].x + r*cw, layout[i].y + (LOAD_BINS-1-l)*ch, cw-1, ch-1, c);
        }
      } else {
        tft.fillRect(layout[i].x, layout[i].y, 150, 20, 0);
        tft.setCursor(layout[i].x, layout[i].y);
        tft.setTextColor(layout[i].params[2]);
        tft.setTextSize(layout[i].font);
        tft.print(layout[i].label); tft.print(val, 1);
        if (layout[i].mode == MODE_BAR) tft.fillRect(layout[i].x, layout[i].y+15, fmap(val, layout[i].value_min, layout[i].value_max, 0, layout[i].params[3]), layout[i].params[4], layout[i].params[2]);
      }
    }
  }
}
