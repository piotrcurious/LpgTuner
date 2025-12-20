// TFT_eSPI library for ESP32
#include <TFT_eSPI.h>
#include <SPI.h>

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

// MAP sensor variables
float currentMAP = 0; // In kPa
#define MAP_MIN_VOLTAGE 0.5
#define MAP_MAX_VOLTAGE 4.5
#define MAP_MIN_KPA 20.0
#define MAP_MAX_KPA 250.0

// Heat map data structures
#define RPM_BINS 10
#define LOAD_BINS 8

// RPM bin edges (in RPM)
float rpmBinEdges[RPM_BINS + 1] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};

// Load bin edges (in kPa)
float loadBinEdges[LOAD_BINS + 1] = {20, 40, 60, 80, 100, 120, 160, 200, 250};

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
    // Widget 1: Temperature
    {
      "Temp01",
      1, // MAX6675
      "Temp:",
      10, 10, 2, 500, MODE_DEC,
      0.0, 1023.0,
      {0, 1023, ORANGE, 120, 20, 0, 0, 0, 0, 0}
    },
    // Widget 2: Lambda
    {
      "lambda",
      2, // Analog input
      "Lambda:",
      170, 10, 2, 100, MODE_DEC,
      0.0, 3.3,
      {0, 1023, CYAN, 120, 20, 0, 0, 0, 0, 0}
    },
    // Widget 3: RPM
    {
      "rpm",
      3, // RPM sensor
      "RPM:",
      10, 40, 2, 100, MODE_DEC,
      0, 10000,
      {0, 10000, GREEN, 120, 20, 0, 0, 0, 0, 0}
    },
    // Widget 4: MAP
    {
      "map",
      4, // MAP sensor
      "MAP:",
      170, 40, 2, 100, MODE_DEC,
      0, 250,
      {0, 250, YELLOW, 120, 20, 0, 0, 0, 0, 0}
    },
    // Widget 5: Temperature bar
    {
      "Temp01",
      1,
      "",
      10, 80, 1, 500, MODE_BAR_FADE,
      20.0, 800.0,
      {20, 800, ORANGE, 300, 8, 20, 50, 0, 0, 0}
    },
    // Widget 6: RPM gauge
    {
      "rpm",
      3,
      "",
      120, 110, 1, 100, MODE_GAUGE,
      0, 10000,
      {0, 10000, GREEN, 80, 80, 50, 0, 0, 0, 0}
    }
  },
  // Layout 2: Graphs
  {
    // Widget 1: Temp label
    {
      "Temp01",
      1,
      "Temp:",
      10, 10, 1, 500, MODE_DEC,
      0, 1023,
      {0, 1023, ORANGE, 80, 16, 0, 0, 0, 0, 0}
    },
    // Widget 2: Lambda label
    {
      "lambda",
      2,
      "Lam:",
      170, 10, 1, 100, MODE_DEC,
      0, 3.3,
      {0, 3.3, CYAN, 80, 16, 0, 0, 0, 0, 0}
    },
    // Widget 3: Temperature graph
    {
      "Temp01",
      1,
      "",
      10, 40, 1, 200, MODE_GRAPH_0,
      30.0, 800.0,
      {30, 800, RED, 140, 80, 0, 0, 0, 0, 0}
    },
    // Widget 4: Lambda graph
    {
      "lambda",
      2,
      "",
      170, 40, 1, 100, MODE_GRAPH_1,
      0, 3.3,
      {0, 3.3, CYAN, 140, 80, 0, 0, 0, 0, 0}
    },
    // Widget 5: RPM bar
    {
      "rpm",
      3,
      "RPM",
      10, 140, 1, 100, MODE_BAR,
      0, 10000,
      {0, 10000, GREEN, 300, 20, 0, 0, 0, 0, 0}
    },
    // Widget 6: MAP bar
    {
      "map",
      4,
      "MAP",
      10, 180, 1, 100, MODE_BAR,
      0, 250,
      {0, 250, YELLOW, 300, 20, 0, 0, 0, 0, 0}
    }
  },
  // Layout 3: Temperature Heat Map
  {
    // Widget 1: Heat map (takes most of the screen)
    {
      "tempmap",
      1, // Source 1 for temp data collection
      "",
      10, 30, 1, 500, MODE_HEATMAP_TEMP,
      0, 850,
      {0, 850, WHITE, 280, 160, 0, 0, 0, 0, 0}  // Adjusted width to leave room for labels
    },
    // Widget 2: Title
    {
      "title",
      0,
      "Temperature Map (C)",
      50, 5, 1, 10000, MODE_DEC,
      0, 0,
      {0, 0, YELLOW, 200, 16, 0, 0, 0, 0, 0}
    },
    // Widget 3: Current Temp
    {
      "temp",
      1,
      "T:",
      10, 215, 1, 500, MODE_DEC,
      0, 1000,
      {0, 1000, ORANGE, 70, 16, 0, 0, 0, 0, 0}
    },
    // Widget 4: Current RPM
    {
      "rpm",
      3,
      "RPM:",
      90, 215, 1, 100, MODE_DEC,
      0, 10000,
      {0, 10000, GREEN, 90, 16, 0, 0, 0, 0, 0}
    },
    // Widget 5: Current MAP
    {
      "map",
      4,
      "MAP:",
      190, 215, 1, 100, MODE_DEC,
      0, 250,
      {0, 250, YELLOW, 90, 16, 0, 0, 0, 0, 0}
    },
    // Widget 6: Empty
    {"", 0, "", 0, 0, 1, 10000, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
  },
  // Layout 4: Lambda Heat Map
  {
    // Widget 1: Heat map (takes most of the screen)
    {
      "lambdamap",
      2, // Source 2 for lambda data collection
      "",
      10, 30, 1, 500, MODE_HEATMAP_LAMBDA,
      0.5, 1.5,
      {0.5, 1.5, WHITE, 280, 160, 0, 0, 0, 0, 0}  // Adjusted width
    },
    // Widget 2: Title
    {
      "title",
      0,
      "Lambda Map",
      90, 5, 1, 10000, MODE_DEC,
      0, 0,
      {0, 0, CYAN, 140, 16, 0, 0, 0, 0, 0}
    },
    // Widget 3: Current Lambda
    {
      "lambda",
      2,
      "L:",
      10, 215, 1, 100, MODE_DEC,
      0, 3.3,
      {0, 3.3, CYAN, 70, 16, 0, 0, 0, 0, 0}
    },
    // Widget 4: Current RPM
    {
      "rpm",
      3,
      "RPM:",
      90, 215, 1, 100, MODE_DEC,
      0, 10000,
      {0, 10000, GREEN, 90, 16, 0, 0, 0, 0, 0}
    },
    // Widget 5: Current MAP
    {
      "map",
      4,
      "MAP:",
      190, 215, 1, 100, MODE_DEC,
      0, 250,
      {0, 250, YELLOW, 90, 16, 0, 0, 0, 0, 0}
    },
    // Widget 6: Empty
    {"", 0, "", 0, 0, 1, 10000, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
  }
};

#define NUM_LAYOUTS (sizeof(layouts) / sizeof(layouts[0]))
#define NUM_WIDGETS (sizeof(layouts[0]) / sizeof(layouts[0][0]))

// Rolling graph buffers
#define BUFFER0_SIZE 140
float buffer0[BUFFER0_SIZE];
#define BUFFER1_SIZE 140
float buffer1[BUFFER1_SIZE];

int bufferIndex = 0;
unsigned long lastUpdate[NUM_WIDGETS];
float currentValue[NUM_LAYOUTS][NUM_WIDGETS];
int currentLayout = 0;
unsigned long lastLayoutChange = 0;
unsigned long layoutChangeInterval = 8000;

// RPM interrupt handler
void IRAM_ATTR rpmPulseISR() {
  rpmPulseCount++;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Get RPM bin index
int getRPMBin(float rpm) {
  for (int i = 0; i < RPM_BINS; i++) {
    if (rpm >= rpmBinEdges[i] && rpm < rpmBinEdges[i + 1]) {
      return i;
    }
  }
  return RPM_BINS - 1; // Return last bin if out of range
}

// Get Load bin index
int getLoadBin(float load) {
  for (int i = 0; i < LOAD_BINS; i++) {
    if (load >= loadBinEdges[i] && load < loadBinEdges[i + 1]) {
      return i;
    }
  }
  return LOAD_BINS - 1;
}

// Update heat maps with current readings
void updateHeatMaps() {
  if (currentRPM > 500) { // Only log when engine is running
    int rpmBin = getRPMBin(currentRPM);
    int loadBin = getLoadBin(currentMAP);
    
    // Get current sensor readings from appropriate layout widgets
    // Use layout 0 which has the main sensor readings
    float temp = currentValue[0][0]; // Temperature from widget 0 in layout 0
    float lambda = currentValue[0][1]; // Lambda from widget 1 in layout 0
    
    // Validate readings before updating map
    if (temp > 0 && temp < 1000 && lambda > 0 && lambda < 5.0) {
      // Running average
      if (sampleCount[rpmBin][loadBin] == 0) {
        tempMap[rpmBin][loadBin] = temp;
        lambdaMap[rpmBin][loadBin] = lambda;
        sampleCount[rpmBin][loadBin] = 1;
      } else {
        float alpha = 0.1; // Smoothing factor
        tempMap[rpmBin][loadBin] = tempMap[rpmBin][loadBin] * (1 - alpha) + temp * alpha;
        lambdaMap[rpmBin][loadBin] = lambdaMap[rpmBin][loadBin] * (1 - alpha) + lambda * alpha;
        sampleCount[rpmBin][loadBin]++;
      }
    }
  }
}

// Get color for temperature (blue = cold, red = hot)
uint16_t getTempColor(float temp, float minTemp, float maxTemp) {
  if (temp < minTemp) temp = minTemp;
  if (temp > maxTemp) temp = maxTemp;
  
  float normalized = (temp - minTemp) / (maxTemp - minTemp);
  
  // Blue -> Cyan -> Green -> Yellow -> Orange -> Red
  if (normalized < 0.2) {
    return tft.color565(0, 0, 255 * (1 - normalized / 0.2) + 128 * (normalized / 0.2));
  } else if (normalized < 0.4) {
    float t = (normalized - 0.2) / 0.2;
    return tft.color565(0, 255 * t, 128 + 127 * (1 - t));
  } else if (normalized < 0.6) {
    float t = (normalized - 0.4) / 0.2;
    return tft.color565(255 * t, 255, 0);
  } else if (normalized < 0.8) {
    float t = (normalized - 0.6) / 0.2;
    return tft.color565(255, 255 * (1 - t), 0);
  } else {
    float t = (normalized - 0.8) / 0.2;
    return tft.color565(255, 0, 0);
  }
}

// Get color for lambda (rich = red, stoich = green, lean = blue)
uint16_t getLambdaColor(float lambda) {
  if (lambda < 0.8) lambda = 0.8;
  if (lambda > 1.2) lambda = 1.2;
  
  if (lambda < 1.0) {
    // Rich (red to green)
    float t = (lambda - 0.8) / 0.2;
    return tft.color565(255, 255 * t, 0);
  } else {
    // Lean (green to blue)
    float t = (lambda - 1.0) / 0.2;
    return tft.color565(0, 255 * (1 - t), 255 * t);
  }
}

void displayHeatMapTemp(WidgetElement widget, float value) {
  // Get parameters from widget
  int cellWidth = widget.params[3] / RPM_BINS;   // Divide total width by RPM bins
  int cellHeight = widget.params[4] / LOAD_BINS;  // Divide total height by LOAD bins
  int startX = widget.x;
  int startY = widget.y;
  
  // Draw cells
  for (int rpm = 0; rpm < RPM_BINS; rpm++) {
    for (int load = 0; load < LOAD_BINS; load++) {
      int x = startX + rpm * cellWidth;
      int y = startY + (LOAD_BINS - 1 - load) * cellHeight;
      
      uint16_t color;
      if (sampleCount[rpm][load] > 0) {
        color = getTempColor(tempMap[rpm][load], widget.value_min, widget.value_max);
      } else {
        color = DARKGREY; // No data
      }
      
      tft.fillRect(x, y, cellWidth - 1, cellHeight - 1, color);
      
      // Draw current position indicator
      int currentRPMBin = getRPMBin(currentRPM);
      int currentLoadBin = getLoadBin(currentMAP);
      if (rpm == currentRPMBin && load == currentLoadBin && currentRPM > 500) {
        tft.drawRect(x, y, cellWidth - 1, cellHeight - 1, WHITE);
        tft.drawRect(x + 1, y + 1, cellWidth - 3, cellHeight - 3, WHITE);
      }
    }
  }
  
  // Draw axis labels
  tft.setTextSize(1);
  tft.setTextColor(widget.params[2], BLACK);  // Use widget color
  
  // Y-axis (Load)
  for (int i = 0; i < LOAD_BINS; i += 2) {
    int y = startY + (LOAD_BINS - 1 - i) * cellHeight + cellHeight / 2 - 4;
    tft.setCursor(startX + RPM_BINS * cellWidth + 2, y);
    tft.print((int)loadBinEdges[i]);
  }
  
  // X-axis (RPM)
  for (int i = 0; i < RPM_BINS; i += 2) {
    int x = startX + i * cellWidth;
    tft.setCursor(x, startY + LOAD_BINS * cellHeight + 2);
    tft.print((int)(rpmBinEdges[i] / 1000));
    tft.print("k");
  }
}

void displayHeatMapLambda(WidgetElement widget, float value) {
  // Get parameters from widget
  int cellWidth = widget.params[3] / RPM_BINS;
  int cellHeight = widget.params[4] / LOAD_BINS;
  int startX = widget.x;
  int startY = widget.y;
  
  // Draw cells
  for (int rpm = 0; rpm < RPM_BINS; rpm++) {
    for (int load = 0; load < LOAD_BINS; load++) {
      int x = startX + rpm * cellWidth;
      int y = startY + (LOAD_BINS - 1 - load) * cellHeight;
      
      uint16_t color;
      if (sampleCount[rpm][load] > 0) {
        color = getLambdaColor(lambdaMap[rpm][load]);
      } else {
        color = DARKGREY;
      }
      
      tft.fillRect(x, y, cellWidth - 1, cellHeight - 1, color);
      
      // Draw current position
      int currentRPMBin = getRPMBin(currentRPM);
      int currentLoadBin = getLoadBin(currentMAP);
      if (rpm == currentRPMBin && load == currentLoadBin && currentRPM > 500) {
        tft.drawRect(x, y, cellWidth - 1, cellHeight - 1, WHITE);
        tft.drawRect(x + 1, y + 1, cellWidth - 3, cellHeight - 3, WHITE);
      }
    }
  }
  
  // Draw axis labels
  tft.setTextSize(1);
  tft.setTextColor(widget.params[2], BLACK);  // Use widget color
  
  for (int i = 0; i < LOAD_BINS; i += 2) {
    int y = startY + (LOAD_BINS - 1 - i) * cellHeight + cellHeight / 2 - 4;
    tft.setCursor(startX + RPM_BINS * cellWidth + 2, y);
    tft.print((int)loadBinEdges[i]);
  }
  
  for (int i = 0; i < RPM_BINS; i += 2) {
    int x = startX + i * cellWidth;
    tft.setCursor(x, startY + LOAD_BINS * cellHeight + 2);
    tft.print((int)(rpmBinEdges[i] / 1000));
    tft.print("k");
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
  tft.setTextSize(1);
  tft.setCursor(90, 130);
  tft.print("Initializing...");
  
  delay(1500);
  tft.fillScreen(BLACK);
  
  // Initialize heat maps
  for (int i = 0; i < RPM_BINS; i++) {
    for (int j = 0; j < LOAD_BINS; j++) {
      tempMap[i][j] = 0;
      lambdaMap[i][j] = 1.0;
      sampleCount[i][j] = 0;
    }
  }
}

void updateDisplay() {
  WidgetElement* layout = layouts[currentLayout];

  for (int i = 0; i < NUM_WIDGETS; i++) {
    WidgetElement widget = layout[i];
    
    if (widget.name == "") continue; // Skip empty widgets

    if (millis() - lastUpdate[i] >= widget.rate) {
      lastUpdate[i] = millis();
      currentValue[currentLayout][i] = readWidget(widget.source);

      switch (widget.mode) {
        case MODE_HEX:
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);              
          tft.print("0x");
          tft.print((int)currentValue[currentLayout][i], HEX);
          break;
          
        case MODE_DEC:
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);
          if (widget.source == 3) { // RPM - no decimals
            tft.print((int)currentValue[currentLayout][i]);
          } else if (widget.source == 4) { // MAP - 1 decimal
            tft.print(currentValue[currentLayout][i], 1);
          } else {
            tft.print(currentValue[currentLayout][i], 2);
          }
          break;
          
        case MODE_BAR:
          clearArea(widget);
          displayBar(widget, currentValue[currentLayout][i]);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          break;

        case MODE_BAR_FADE:
          clearAreaRate(widget, widget.params[6]);
          displayBarRate(widget, currentValue[currentLayout][i]);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          break;
          
        case MODE_GRAPH_0:
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          displayGraphx30(widget, currentValue[currentLayout][i]);
          break;

        case MODE_GRAPH_1:
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          displayGraphx31(widget, currentValue[currentLayout][i]);
          break;

        case MODE_GAUGE:
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          displayGauge(widget, currentValue[currentLayout][i]);
          break;
          
        case MODE_HEATMAP_TEMP:
          // Only clear and redraw when widget updates (respects widget.rate)
          clearArea(widget);
          displayHeatMapTemp(widget, currentValue[currentLayout][i]);
          break;
          
        case MODE_HEATMAP_LAMBDA:
          // Only clear and redraw when widget updates (respects widget.rate)
          clearArea(widget);
          displayHeatMapLambda(widget, currentValue[currentLayout][i]);
          break;
          
        default:
          tft.print("?");
          break;
      }
    }
  }
}

float readWidget(uint16_t source) {
  switch (source) {
    case 1: // MAX6675 thermocouple
      {
        if (millis() - last_conversion_time >= MAX6675_CONVERSION_RATE) { 
          int status = thermoCouple.read();
          last_conversion_time = millis();
        }
        return thermoCouple.getTemperature();
      }
      
    case 2: // Lambda sensor (analog)
      {
        float voltage = analogRead(analogInPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0);
        // Convert voltage to lambda (adjust based on your sensor)
        // This is a placeholder - calibrate for your sensor
        float lambda = voltage / 3.3; // Simple linear mapping
        return lambda;
      }
      
    case 3: // RPM
      return currentRPM;
      
    case 4: // MAP sensor
      return currentMAP;
      
    default:
      return random(0, 1024);
  }
}

void calculateRPM() {
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

void readMAP() {
  int rawValue = analogRead(mapSensorPin);
  float voltage = rawValue * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0);
  
  // Convert voltage to kPa (adjust based on your sensor)
  currentMAP = fmap(voltage, MAP_MIN_VOLTAGE, MAP_MAX_VOLTAGE, MAP_MIN_KPA, MAP_MAX_KPA);
  
  if (currentMAP < MAP_MIN_KPA) currentMAP = MAP_MIN_KPA;
  if (currentMAP > MAP_MAX_KPA) currentMAP = MAP_MAX_KPA;
}

void clearArea(WidgetElement widget) {
  int width = widget.params[3];
  int height = widget.params[4];
  
  // For heat maps, need to clear the entire area including axis labels
  if (widget.mode == MODE_HEATMAP_TEMP || widget.mode == MODE_HEATMAP_LAMBDA) {
    // Add space for axis labels
    tft.fillRect(widget.x, widget.y, width + 30, height + 15, BLACK);
  } else {
    tft.fillRect(widget.x, widget.y, width, height + 1, BLACK);
  }
}

void clearAreaRate(WidgetElement widget, int rate) {
  int width = widget.params[3];
  int height = widget.params[4];
  fillRectWithRate(widget.x, widget.y, width, height + 1, BLACK, rate);
}

void fillRectWithRate(int x, int y, int w, int h, uint16_t color, int rate) {
  for (int py = y; py < y + h; py++) {
    for (int px = x; px < x + w; px++) {
      if (random(100) > rate) {
        tft.drawPixel(px, py, color);
      }
    }
  }
}

void displayBar(WidgetElement widget, float value) {
  float min = widget.value_min;
  float max = widget.value_max;
  int color = widget.params[2];
  int width = widget.params[3];
  int height = widget.params[4];

  int bar = fmap(value, min, max, 0, width);
  if (bar < 0) bar = 0;
  if (bar > width) bar = width;
  tft.fillRect(widget.x, widget.y, bar, height, color);
}

void displayBarRate(WidgetElement widget, float value) {
  float min = widget.value_min;
  float max = widget.value_max;
  int color = widget.params[2];
  int width = widget.params[3];
  int height = widget.params[4];
  int rate = widget.params[5];

  int bar = fmap(value, min, max, 0, width);
  if (bar < 0) bar = 0;
  if (bar > width) bar = width;
  fillRectWithRate(widget.x, widget.y, bar, height, color, rate);
}

void displayGraphx30(WidgetElement widget, float value) {
  float min = widget.value_min;
  float max = widget.value_max;
  int color = widget.params[2];
  int width = widget.params[3];
  int height = widget.params[4];

  int graph = fmap(value, min, max, 0, height);
  if (graph < 0) graph = 0;
  if (graph > height) graph = height;

  for (uint16_t i = 0; i < BUFFER0_SIZE - 1; i++) {
    buffer0[i] = buffer0[i + 1];
  }
  buffer0[BUFFER0_SIZE - 1] = graph;

  for (uint16_t i = 0; i < BUFFER0_SIZE; i++) {
    int x = widget.x + i;
    int y = widget.y + height - buffer0[i];
    tft.drawPixel(x, y, color);
  }
}

void displayGraphx31(WidgetElement widget, float value) {
  float min = widget.value_min;
  float max = widget.value_max;
  int color = widget.params[2];
  int width = widget.params[3];
  int height = widget.params[4];

  int graph = fmap(value, min, max, 0, height);
  if (graph < 0) graph = 0;
  if (graph > height) graph = height;

  for (uint16_t i = 0; i < BUFFER1_SIZE - 1; i++) {
    buffer1[i] = buffer1[i + 1];
  }
  buffer1[BUFFER1_SIZE - 1] = graph;

  for (uint16_t i = 0; i < BUFFER1_SIZE; i++) {
    int x = widget.x + i;
    int y = widget.y + height - buffer1[i];
    tft.drawPixel(x, y, color);
  }
}

void displayGauge(WidgetElement widget, float value) {
  float min = widget.value_min;
  float max = widget.value_max;
  int color = widget.params[2];
  int width = widget.params[3];
  int height = widget.params[4];
  int radius = widget.params[5];

  int gauge = fmap(value, min, max, -90, 90);
  float angle = gauge * PI / 180.0;

  int cx = widget.x + width / 2;
  int cy = widget.y + height / 2;
  int x = cx + radius * cos(angle);
  int y = cy + radius * sin(angle);

  tft.drawCircle(cx, cy, radius, color);
  tft.drawLine(cx, cy, x, y, color);
}

void setup() {
  Serial.begin(115200);
  
  initDisplay();
  
  currentLayout = 0;
  randomSeed(analogRead(analogInPin));
  
  // Initialize SPI for MAX6675
  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  
  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  // Setup RPM interrupt
  pinMode(rpmInputPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmInputPin), rpmPulseISR, FALLING);
  
  Serial.println("Lambda Angel v2 initialized");
}

void switch_layout() {
  if (millis() - lastLayoutChange >= layoutChangeInterval) {
    lastLayoutChange = millis();
    currentLayout++;
    
    if (currentLayout >= NUM_LAYOUTS) {
      currentLayout = 0;
    }
    
    tft.fillScreen(BLACK);
    
    // Reset update timers for new layout
    for (int i = 0; i < NUM_WIDGETS; i++) {
      lastUpdate[i] = 0;
    }
    
    Serial.print("Switched to layout ");
    Serial.println(currentLayout);
  }
}

void loop() {
  // Read sensors
  calculateRPM();
  readMAP();
  
  // Update heat maps
  updateHeatMaps();
  
  // Uncomment to enable automatic layout switching
  switch_layout();
  
  // Update display
  updateDisplay();
  
  // Optional: Print debug info
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    Serial.print("RPM: ");
    Serial.print(currentRPM, 0);
    Serial.print(" | MAP: ");
    Serial.print(currentMAP, 1);
    Serial.print(" kPa | Temp: ");
    Serial.print(currentValue[0][0], 1);
    Serial.print(" | Lambda: ");
    Serial.println(currentValue[0][1], 3);
    lastDebug = millis();
  }
}
