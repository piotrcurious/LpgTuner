// TFT_eSPI library for ESP32
#include <TFT_eSPI.h>
#include <SPI.h>
#include "CommonLogic.h"

// Display object
TFT_eSPI tft = TFT_eSPI();

// Display dimensions
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// Color definitions (16-bit RGB565)
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
const int selectPin = 5; // CS pin for MAX6675

MAX6675 thermoCouple(selectPin);
uint32_t last_conversion_time = 0; 
#define MAX6675_CONVERSION_RATE 200

// Input pins
const int analogInPin = 34;      // Lambda sensor (ADC1_CH6)
const int mapSensorPin = 35;     // MAP sensor (ADC1_CH7)
const int rpmInputPin = 32;      // RPM input (pulse counter)

#define MAX_ANALOG_VOLTAGE_A0 3.3
#define ADC_RESOLUTION_A0 4096

// RPM calculation variables
volatile unsigned long rpmPulseCount = 0;
unsigned long lastRPMCalcTime = 0;
float currentRPM = 0;
#define RPM_CALC_INTERVAL 100 // Calculate RPM every 100ms
#define PULSES_PER_REV 2 // Adjust based on your sensor

// Sensor reading globals
float currentTemp = 0;
float currentLambda = 0;
float currentMAP = 0; // In kPa

// Heat maps
float tempMap[RPM_BINS][LOAD_BINS];
float lambdaMap[RPM_BINS][LOAD_BINS];
int sampleCount[RPM_BINS][LOAD_BINS]; // Track samples per bin

// Define the widget element struct
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

// Widget mode definitions
#define MODE_HEX 0
#define MODE_DEC 1
#define MODE_BAR 2
#define MODE_BAR_FADE 0x21
#define MODE_GRAPH_0 0x30
#define MODE_GRAPH_1 0x31
#define MODE_GAUGE 4
#define MODE_HEATMAP_TEMP 5
#define MODE_HEATMAP_LAMBDA 6

// Define the layout array
WidgetElement layouts[][6] = {
  // Layout 1: Main dashboard
  {
    {"Temp01", 1, "Temp:", 10, 10, 2, 500, MODE_DEC, 0.0, 1023.0, {0, 1023, ORANGE, 120, 20}},
    {"lambda", 2, "Lambda:", 170, 10, 2, 100, MODE_DEC, 0.0, 3.3, {0, 1023, CYAN, 120, 20}},
    {"rpm", 3, "RPM:", 10, 40, 2, 100, MODE_DEC, 0, 10000, {0, 10000, GREEN, 120, 20}},
    {"map", 4, "MAP:", 170, 40, 2, 100, MODE_DEC, 0, 250, {0, 250, YELLOW, 120, 20}},
    {"Temp01", 1, "", 10, 80, 1, 500, MODE_BAR_FADE, 20.0, 800.0, {20, 800, ORANGE, 300, 8, 20, 50}},
    {"rpm", 3, "", 120, 110, 1, 100, MODE_GAUGE, 0, 10000, {0, 10000, GREEN, 80, 80, 50}}
  },
  // Layout 2: Graphs
  {
    {"Temp01", 1, "Temp:", 10, 10, 1, 500, MODE_DEC, 0, 1023, {0, 1023, ORANGE, 80, 16}},
    {"lambda", 2, "Lam:", 170, 10, 1, 100, MODE_DEC, 0, 3.3, {0, 3.3, CYAN, 80, 16}},
    {"Temp01", 1, "", 10, 40, 1, 200, MODE_GRAPH_0, 30.0, 800.0, {30, 800, RED, 140, 80}},
    {"lambda", 2, "", 170, 40, 1, 100, MODE_GRAPH_1, 0, 3.3, {0, 3.3, CYAN, 140, 80}},
    {"rpm", 3, "RPM", 10, 140, 1, 100, MODE_BAR, 0, 10000, {0, 10000, GREEN, 300, 20}},
    {"map", 4, "MAP", 10, 180, 1, 100, MODE_BAR, 0, 250, {0, 250, YELLOW, 300, 20}}
  },
  // Layout 3: Temperature Heat Map
  {
    {"tempmap", 1, "", 10, 30, 1, 500, MODE_HEATMAP_TEMP, 0, 850, {0, 850, WHITE, 280, 160}},
    {"title", 0, "Temperature Map (C)", 50, 5, 1, 10000, MODE_DEC, 0, 0, {0, 0, YELLOW, 200, 16}},
    {"temp", 1, "T:", 10, 215, 1, 500, MODE_DEC, 0, 1000, {0, 1000, ORANGE, 70, 16}},
    {"rpm", 3, "RPM:", 90, 215, 1, 100, MODE_DEC, 0, 10000, {0, 10000, GREEN, 90, 16}},
    {"map", 4, "MAP:", 190, 215, 1, 100, MODE_DEC, 0, 250, {0, 250, YELLOW, 90, 16}},
    {"", 0, "", 0, 0, 1, 10000, 0, 0, 0, {0}}
  },
  // Layout 4: Lambda Heat Map
  {
    {"lambdamap", 2, "", 10, 30, 1, 500, MODE_HEATMAP_LAMBDA, 0.5, 1.5, {0.5, 1.5, WHITE, 280, 160}},
    {"title", 0, "Lambda Map", 90, 5, 1, 10000, MODE_DEC, 0, 0, {0, 0, CYAN, 140, 16}},
    {"lambda", 2, "L:", 10, 215, 1, 100, MODE_DEC, 0, 3.3, {0, 3.3, CYAN, 70, 16}},
    {"rpm", 3, "RPM:", 90, 215, 1, 100, MODE_DEC, 0, 10000, {0, 10000, GREEN, 90, 16}},
    {"map", 4, "MAP:", 190, 215, 1, 100, MODE_DEC, 0, 250, {0, 250, YELLOW, 90, 16}},
    {"", 0, "", 0, 0, 1, 10000, 0, 0, 0, {0}}
  }
};

#define NUM_LAYOUTS (sizeof(layouts) / sizeof(layouts[0]))
#define NUM_WIDGETS (sizeof(layouts[0]) / sizeof(layouts[0][0]))

// Rolling graph buffers
#define BUFFER0_SIZE 140
float buffer0[BUFFER0_SIZE];
#define BUFFER1_SIZE 140
float buffer1[BUFFER1_SIZE];

unsigned long lastUpdate[NUM_WIDGETS];
int currentLayout = 0;
unsigned long lastLayoutChange = 0;
unsigned long layoutChangeInterval = 8000;

// RPM interrupt handler
void IRAM_ATTR rpmPulseISR() {
  rpmPulseCount++;
}

// Update heat maps with current readings
void updateHeatMaps() {
  if (currentRPM > 500) { // Only log when engine is running
    int rpmBin = getRPMBin(currentRPM);
    int loadBin = getLoadBin(currentMAP);
    
    // Validate readings before updating map
    if (currentTemp > 0 && currentTemp < 1000 && currentLambda > 0 && currentLambda < 5.0) {
      // Running average
      if (sampleCount[rpmBin][loadBin] == 0) {
        tempMap[rpmBin][loadBin] = currentTemp;
        lambdaMap[rpmBin][loadBin] = currentLambda;
        sampleCount[rpmBin][loadBin] = 1;
      } else {
        float alpha = 0.1; // Smoothing factor
        tempMap[rpmBin][loadBin] = tempMap[rpmBin][loadBin] * (1 - alpha) + currentTemp * alpha;
        lambdaMap[rpmBin][loadBin] = lambdaMap[rpmBin][loadBin] * (1 - alpha) + currentLambda * alpha;
        sampleCount[rpmBin][loadBin]++;
      }
    }
  }
}

void clearArea(WidgetElement widget) {
  int width = widget.params[3];
  int height = widget.params[4];
  if (widget.mode == MODE_HEATMAP_TEMP || widget.mode == MODE_HEATMAP_LAMBDA) {
    tft.fillRect(widget.x, widget.y, width + 30, height + 15, BLACK);
  } else {
    tft.fillRect(widget.x, widget.y, width, height + 1, BLACK);
  }
}

void displayBar(WidgetElement widget, float value) {
  int bar = fmap(value, widget.value_min, widget.value_max, 0, widget.params[3]);
  if (bar < 0) bar = 0; if (bar > widget.params[3]) bar = widget.params[3];
  tft.fillRect(widget.x, widget.y, bar, widget.params[4], widget.params[2]);
}

void displayGauge(WidgetElement widget, float value) {
  int gauge = fmap(value, widget.value_min, widget.value_max, -90, 90);
  float angle = gauge * PI / 180.0;
  int cx = widget.x + widget.params[3] / 2;
  int cy = widget.y + widget.params[4] / 2;
  int radius = widget.params[5];
  int x = cx + radius * cos(angle);
  int y = cy + radius * sin(angle);
  tft.drawCircle(cx, cy, radius, widget.params[2]);
  tft.drawLine(cx, cy, x, y, widget.params[2]);
}

void displayGraph(WidgetElement widget, float value, float* buffer, int bufferSize) {
  int h = widget.params[4];
  int graph = fmap(value, widget.value_min, widget.value_max, 0, h);
  if (graph < 0) graph = 0; if (graph > h) graph = h;
  for (int i = 0; i < bufferSize - 1; i++) buffer[i] = buffer[i + 1];
  buffer[bufferSize - 1] = graph;
  for (int i = 0; i < bufferSize; i++) {
    tft.drawPixel(widget.x + i, widget.y + h - (int)buffer[i], widget.params[2]);
  }
}

void displayHeatMapTemp(WidgetElement widget) {
  int cellWidth = widget.params[3] / RPM_BINS;
  int cellHeight = widget.params[4] / LOAD_BINS;
  for (int rpm = 0; rpm < RPM_BINS; rpm++) {
    for (int load = 0; load < LOAD_BINS; load++) {
      int x = widget.x + rpm * cellWidth;
      int y = widget.y + (LOAD_BINS - 1 - load) * cellHeight;
      uint16_t color = (sampleCount[rpm][load] > 0) ? getTempColor(tempMap[rpm][load], widget.value_min, widget.value_max) : DARKGREY;
      tft.fillRect(x, y, cellWidth - 1, cellHeight - 1, color);
      if (rpm == getRPMBin(currentRPM) && load == getLoadBin(currentMAP) && currentRPM > 500) {
        tft.drawRect(x, y, cellWidth - 1, cellHeight - 1, WHITE);
      }
    }
  }
}

void displayHeatMapLambda(WidgetElement widget) {
  int cellWidth = widget.params[3] / RPM_BINS;
  int cellHeight = widget.params[4] / LOAD_BINS;
  for (int rpm = 0; rpm < RPM_BINS; rpm++) {
    for (int load = 0; load < LOAD_BINS; load++) {
      int x = widget.x + rpm * cellWidth;
      int y = widget.y + (LOAD_BINS - 1 - load) * cellHeight;
      uint16_t color = (sampleCount[rpm][load] > 0) ? getLambdaColor(lambdaMap[rpm][load]) : DARKGREY;
      tft.fillRect(x, y, cellWidth - 1, cellHeight - 1, color);
      if (rpm == getRPMBin(currentRPM) && load == getLoadBin(currentMAP) && currentRPM > 500) {
        tft.drawRect(x, y, cellWidth - 1, cellHeight - 1, WHITE);
      }
    }
  }
}

void initDisplay() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(60, 100);
  tft.print("Lambda Angel v2");
  delay(1500);
  tft.fillScreen(BLACK);
  for (int i = 0; i < RPM_BINS; i++) {
    for (int j = 0; j < LOAD_BINS; j++) {
      tempMap[i][j] = 0; lambdaMap[i][j] = 1.0; sampleCount[i][j] = 0;
    }
  }
}

void readSensors() {
  if (millis() - last_conversion_time >= MAX6675_CONVERSION_RATE) {
    thermoCouple.read();
    currentTemp = thermoCouple.getTemperature();
    last_conversion_time = millis();
  }
  currentLambda = (analogRead(analogInPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0)) / 1.0;
  currentMAP = voltageToKpa(analogRead(mapSensorPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0));
  if (millis() - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
    noInterrupts(); unsigned long pulses = rpmPulseCount; rpmPulseCount = 0; interrupts();
    float timeSeconds = (millis() - lastRPMCalcTime) / 1000.0;
    currentRPM = (pulses / PULSES_PER_REV) * (60.0 / timeSeconds);
    lastRPMCalcTime = millis();
  }
}

void updateDisplay() {
  WidgetElement* layout = layouts[currentLayout];
  for (int i = 0; i < NUM_WIDGETS; i++) {
    WidgetElement widget = layout[i];
    if (widget.name == "") continue;
    if (millis() - lastUpdate[i] >= widget.rate) {
      lastUpdate[i] = millis();
      float val = (widget.source == 1) ? currentTemp : (widget.source == 2) ? currentLambda : (widget.source == 3) ? currentRPM : currentMAP;
      switch (widget.mode) {
        case MODE_HEX:
          clearArea(widget);
          tft.setCursor(widget.x, widget.y); tft.setTextColor(widget.params[2]); tft.setTextSize(widget.font);
          tft.print(widget.label); tft.print((int)val, HEX);
          break;
        case MODE_DEC:
          clearArea(widget);
          tft.setCursor(widget.x, widget.y); tft.setTextColor(widget.params[2]); tft.setTextSize(widget.font);
          tft.print(widget.label); tft.print(val, (widget.source == 3) ? 0 : 2);
          break;
        case MODE_BAR:
        case MODE_BAR_FADE:
          clearArea(widget);
          displayBar(widget, val);
          break;
        case MODE_GRAPH_0:
          clearArea(widget);
          displayGraph(widget, val, buffer0, BUFFER0_SIZE);
          break;
        case MODE_GRAPH_1:
          clearArea(widget);
          displayGraph(widget, val, buffer1, BUFFER1_SIZE);
          break;
        case MODE_GAUGE:
          clearArea(widget);
          displayGauge(widget, val);
          break;
        case MODE_HEATMAP_TEMP:
          clearArea(widget);
          displayHeatMapTemp(widget);
          break;
        case MODE_HEATMAP_LAMBDA:
          clearArea(widget);
          displayHeatMapLambda(widget);
          break;
      }
    }
  }
}

void setup() {
  Serial.begin(115200); initDisplay(); SPI.begin(); thermoCouple.begin();
  pinMode(rpmInputPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmInputPin), rpmPulseISR, FALLING);
}

void loop() {
  readSensors(); updateHeatMaps();
  if (millis() - lastLayoutChange >= layoutChangeInterval) {
    lastLayoutChange = millis(); currentLayout = (currentLayout + 1) % NUM_LAYOUTS; tft.fillScreen(BLACK);
    for (int i = 0; i < NUM_WIDGETS; i++) lastUpdate[i] = 0;
  }
  updateDisplay();
}
