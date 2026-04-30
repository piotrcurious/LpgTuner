#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// Custom LGFX configuration for ESP32 and ST7789/ILI9341
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
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;
      cfg.panel_height     = 320;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.bus_shared       = true;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;

// Display dimensions
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// Color definitions
#define BLACK       0x0000
#define WHITE       0xFFFF
#define RED         0xF800
#define GREEN       0x07E0
#define BLUE        0x001F
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define ORANGE      0xFD20
#define DARKGREY    0x7BEF

//--------- INPUTS section
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

// Sensor globals
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

// Heat map data
#define RPM_BINS 10
#define LOAD_BINS 8
float rpmBinEdges[RPM_BINS + 1] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};
float loadBinEdges[LOAD_BINS + 1] = {20, 40, 60, 80, 100, 120, 160, 200, 250};
float tempMap[RPM_BINS][LOAD_BINS];
float lambdaMap[RPM_BINS][LOAD_BINS];
int sampleCount[RPM_BINS][LOAD_BINS];

// Rolling graph buffers
#define BUFFER0_SIZE 140
float buffer0[BUFFER0_SIZE];
#define BUFFER1_SIZE 140
float buffer1[BUFFER1_SIZE];

// Widget system
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

#define MODE_HEX 0
#define MODE_DEC 1
#define MODE_BAR 2
#define MODE_BAR_FADE 0x21
#define MODE_GRAPH_0 0x30
#define MODE_GRAPH_1 0x31
#define MODE_GAUGE 4
#define MODE_HEATMAP_TEMP 5
#define MODE_HEATMAP_LAMBDA 6

WidgetElement layouts[][6] = {
  {
    {"Temp", 1, "T:", 10, 10, 2, 500, MODE_DEC, 0, 1000, {0, 0, ORANGE, 120, 20}},
    {"Lambda", 2, "L:", 170, 10, 2, 100, MODE_DEC, 0, 2, {0, 0, CYAN, 120, 20}},
    {"RPM", 3, "RPM:", 10, 40, 2, 100, MODE_DEC, 0, 10000, {0, 0, GREEN, 120, 20}},
    {"MAP", 4, "MAP:", 170, 40, 2, 100, MODE_DEC, 0, 250, {0, 0, YELLOW, 120, 20}},
    {"TBar", 1, "", 10, 80, 1, 500, MODE_BAR, 0, 1000, {0, 0, ORANGE, 300, 10}},
    {"RGauge", 3, "", 120, 110, 1, 100, MODE_GAUGE, 0, 10000, {0, 0, GREEN, 80, 80, 50}}
  }
};

#define NUM_LAYOUTS (sizeof(layouts) / sizeof(layouts[0]))
#define NUM_WIDGETS (sizeof(layouts[0]) / sizeof(layouts[0][0]))
unsigned long lastUpdate[NUM_WIDGETS];
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
  currentLambda = (analogRead(analogInPin) * (3.3 / 4096.0)) / 3.3;
  float mapV = analogRead(mapSensorPin) * (3.3 / 4096.0);
  currentMAP = fmap(mapV, MAP_MIN_VOLTAGE, MAP_MAX_VOLTAGE, MAP_MIN_KPA, MAP_MAX_KPA);
  if (millis() - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
    noInterrupts();
    unsigned long pulses = rpmPulseCount;
    rpmPulseCount = 0;
    interrupts();
    currentRPM = (pulses / PULSES_PER_REV) * (60.0 / (RPM_CALC_INTERVAL / 1000.0));
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

void displayBar(WidgetElement widget, float value) {
  int bar = fmap(value, widget.value_min, widget.value_max, 0, widget.params[3]);
  tft.fillRect(widget.x, widget.y, bar, widget.params[4], widget.params[2]);
}

void displayGauge(WidgetElement widget, float value) {
  int gauge = fmap(value, widget.value_min, widget.value_max, -90, 90);
  float angle = gauge * PI / 180.0;
  int cx = widget.x + widget.params[3] / 2;
  int cy = widget.y + widget.params[4] / 2;
  int x = cx + widget.params[5] * cos(angle);
  int y = cy + widget.params[5] * sin(angle);
  tft.drawCircle(cx, cy, widget.params[5], widget.params[2]);
  tft.drawLine(cx, cy, x, y, widget.params[2]);
}

void setup() {
  Serial.begin(115200);
  tft.init(); tft.setRotation(1); tft.fillScreen(BLACK);
  SPI.begin(); thermoCouple.begin();
  pinMode(rpmInputPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmInputPin), rpmPulseISR, FALLING);
}

void loop() {
  readSensors();
  updateHeatMaps();
  WidgetElement* layout = layouts[currentLayout];
  for (int i = 0; i < NUM_WIDGETS; i++) {
    if (millis() - lastUpdate[i] >= layout[i].rate) {
      lastUpdate[i] = millis();
      float val = (layout[i].source == 1) ? currentTemp : (layout[i].source == 2) ? currentLambda : (layout[i].source == 3) ? currentRPM : currentMAP;
      tft.fillRect(layout[i].x, layout[i].y, layout[i].params[3] + 20, layout[i].params[4] + 2, BLACK);
      tft.setCursor(layout[i].x, layout[i].y);
      tft.setTextColor(layout[i].params[2]);
      tft.print(layout[i].label); tft.print(val, 1);
      if (layout[i].mode == MODE_BAR) displayBar(layout[i], val);
      else if (layout[i].mode == MODE_GAUGE) displayGauge(layout[i], val);
    }
  }
}
