// Engine Tuning Map Plotting Tool
// Displays injection timing on RPM/Load map with multiple visualization modes
// Optimized for ESP32 with 320x240 TFT_eSPI display
// Button on GPIO 0 (BOOT button) switches visualization modes

#ifdef UNIT_TEST
#include "mock_tft.h"
#include "mock_arduino.h"
#else
#include <TFT_eSPI.h>
#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
#endif

#if defined(ESP32) || defined(UNIT_TEST)
#ifdef UNIT_TEST
#include "mock_arduino.h"
#else
#include <Preferences.h>
#endif
#endif


// Initialize display
TFT_eSPI tft = TFT_eSPI();

// Define pins for sensors and injector
const int throttlePin = 34;      // Throttle position sensor (ADC1)
const int mapPin = 35;           // MAP sensor (ADC1)
const int lambdaPin = 32;        // Lambda probe (ADC1)
const int camPin = 26;           // Cam sensor (interrupt capable)
const int PULSES_PER_REV = 1;    // Pulses per revolution for RPM calculation
const int injectorPin = 27;      // Injector input (interrupt capable)
const int injectorGaugePin = 25; // PWM output for gauge (PWM capable)
const int buttonPin = 0;         // Boot button for mode switching

// Visualization modes
enum VisualizationMode {
  MODE_SCATTER_MAP,      // Original colored dot plot
  MODE_HEAT_MAP,         // Heat map with cells
  MODE_3D_SURFACE,       // Pseudo-3D surface plot
  MODE_AFR_MAP,          // AFR/Lambda visualization
  MODE_DENSITY_MAP,      // Density plot showing operating points
  MODE_COUNT
};

// Overlay modes
enum OverlayMode {
  OVERLAY_NORMAL,        // Show current EMA value
  OVERLAY_LIFETIME,      // Show lifetime average
  OVERLAY_DIFF,          // Show EMA - Lifetime Average (Deviation)
  OVERLAY_COUNT
};

VisualizationMode currentMode = MODE_SCATTER_MAP;
OverlayMode currentOverlay = OVERLAY_NORMAL;

// Heat map data structure (12x9 grid = 108 cells)
const int HEATMAP_COLS = 12;  // RPM divisions
const int HEATMAP_ROWS = 9;   // Load divisions
struct HeatMapCell {
  float sumPulseWidth;   // Lifetime sum for average
  float sumLambda;
  float avgPulseWidth;   // For exponential moving average (EMA)
  int count;             // Lifetime count
  unsigned long lastUpdate;
};
HeatMapCell heatMap[HEATMAP_COLS][HEATMAP_ROWS];

// Sensor and injector variables
float throttlePos = 0;
float mapPressure = 0;
float lambdaValue = 0;
float rpm = 0;
float pulseWidth = 0;
float pulseWidthSmooth = 0;

// Cam sensor interrupt variables
volatile unsigned long lastCamTime = 0;
volatile unsigned long camPeriod = 1000000;
volatile bool newCamData = false;

// Injector interrupt variables
volatile unsigned long injectorOnTime = 0;
volatile unsigned long injectorOffTime = 0;
volatile unsigned long measuredPulseWidth = 0;
volatile bool newPulseData = false;

// Button handling
volatile unsigned long lastButtonPress = 0;
volatile bool buttonPressed = false;

// Display constants
const int DISPLAY_WIDTH = 320;
const int DISPLAY_HEIGHT = 240;
const int CHART_WIDTH = 280;
const int CHART_HEIGHT = 180;
const int CHART_X_OFFSET = 30;
const int CHART_Y_OFFSET = 35;

// Update intervals
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;

// Color definitions
#define COLOR_BACKGROUND TFT_BLACK
#define COLOR_TEXT TFT_WHITE
#define COLOR_AXIS TFT_WHITE
#define COLOR_GRID TFT_DARKGREY

// Forward declarations
void IRAM_ATTR camISR();
void IRAM_ATTR injectorISR();
void IRAM_ATTR buttonISR();
void handleButton();
uint16_t getColorForDiff(float diff);
void saveLifetimeData();
void loadLifetimeData();
void resetLifetimeData();
void initDisplay();
void drawGui();
void readSensors();
void measureInjectorPulseWidth();
void controlInjectorGauge();
void updateVisualization();
void drawScatterMap();
void drawHeatMap();
void draw3DSurface();
void drawAFRMap();
void drawDensityMap();
void drawCurrentPulseWidth();
void drawSensorReadings();
void drawModeIndicator();
void switchVisualizationMode();
void clearHeatMap();
uint16_t getColorForPulseWidth(float pw);
uint16_t getColorForAFR(float lambda);
uint16_t interpolateColor(uint16_t color1, uint16_t color2, float t);

void setup() {
  Serial.begin(115200);
  Serial.println("Engine Tuning Map Tool Starting...");
  
  // Configure pins
  pinMode(throttlePin, INPUT);
  pinMode(mapPin, INPUT);
  pinMode(lambdaPin, INPUT);
  pinMode(camPin, INPUT_PULLUP);
  pinMode(injectorPin, INPUT);
  pinMode(injectorGaugePin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(camPin), camISR, RISING);
  attachInterrupt(digitalPinToInterrupt(injectorPin), injectorISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
  
  // Initialize heat map
  clearHeatMap();
  
  // Load saved data
  loadLifetimeData();

  // Initialize display
  tft.init();
  tft.setRotation(1); // Landscape mode
  initDisplay();
  drawGui();
  
  Serial.println("Initialization complete");
  Serial.println("Press BOOT button to switch visualization modes");
}

void loop() {
  readSensors();
  measureInjectorPulseWidth();
  controlInjectorGauge();
  
  // Handle mode switching
  handleButton();
  
  // Update display at controlled intervals
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = millis();
    updateVisualization();
    drawCurrentPulseWidth();
    drawSensorReadings();
    drawModeIndicator();
  }

  // Periodic save
  static unsigned long lastSaveTime = 0;
  const unsigned long SAVE_INTERVAL = 30000; // Save every 30 seconds
  if (millis() - lastSaveTime >= SAVE_INTERVAL) {
    lastSaveTime = millis();
    saveLifetimeData();
  }
  
  // Optional: Print debug info to serial
  static unsigned long lastSerialPrint = 0;
  if (millis() - lastSerialPrint >= 1000) {
    lastSerialPrint = millis();
    Serial.printf("Mode:%d RPM:%.0f Load:%.1f%% PW:%.2fms Lambda:%.2f\n", 
                  currentMode, rpm, (mapPressure * throttlePos / 100.0), pulseWidth, lambdaValue);
  }
}

void initDisplay() {
  tft.fillScreen(COLOR_BACKGROUND);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
}

void drawGui() {
  tft.fillScreen(COLOR_BACKGROUND);
  
  // Draw title
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, COLOR_BACKGROUND);
  tft.drawString("Engine Tuning Analyzer", 10, 5);
  
  // Draw chart frame
  tft.drawRect(CHART_X_OFFSET - 1, CHART_Y_OFFSET - 1, 
               CHART_WIDTH + 2, CHART_HEIGHT + 2, COLOR_AXIS);
  
  // Draw grid lines
  for (int i = 1; i < 5; i++) {
    int x = CHART_X_OFFSET + (CHART_WIDTH * i / 5);
    int y = CHART_Y_OFFSET + (CHART_HEIGHT * i / 5);
    tft.drawLine(x, CHART_Y_OFFSET, x, CHART_Y_OFFSET + CHART_HEIGHT, COLOR_GRID);
    tft.drawLine(CHART_X_OFFSET, y, CHART_X_OFFSET + CHART_WIDTH, y, COLOR_GRID);
  }
  
  // Draw axis labels
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
  tft.setTextSize(1);
  tft.drawString("RPM", CHART_X_OFFSET + CHART_WIDTH / 2 - 10, DISPLAY_HEIGHT - 10);
  
  // Vertical text for "Load"
  tft.drawString("L", 5, CHART_Y_OFFSET + CHART_HEIGHT / 2 - 20);
  tft.drawString("o", 5, CHART_Y_OFFSET + CHART_HEIGHT / 2 - 10);
  tft.drawString("a", 5, CHART_Y_OFFSET + CHART_HEIGHT / 2);
  tft.drawString("d", 5, CHART_Y_OFFSET + CHART_HEIGHT / 2 + 10);
  
  // Draw scale labels
  tft.setTextSize(1);
  tft.drawString("0", CHART_X_OFFSET - 15, CHART_Y_OFFSET + CHART_HEIGHT - 5);
  tft.drawString("6k", CHART_X_OFFSET + CHART_WIDTH - 10, CHART_Y_OFFSET + CHART_HEIGHT + 5);
  tft.drawString("100", CHART_X_OFFSET - 20, CHART_Y_OFFSET - 5);
}

#ifdef ESP32
portMUX_TYPE camMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;
#endif

void readSensors() {
  // Read analog sensors with averaging
  const int samples = 4;
  int throttleSum = 0, mapSum = 0, lambdaSum = 0;
  
  for (int i = 0; i < samples; i++) {
    throttleSum += analogRead(throttlePin);
    mapSum += analogRead(mapPin);
    lambdaSum += analogRead(lambdaPin);
  }
  
  // Map to meaningful values
  throttlePos = map(throttleSum / samples, 0, 4095, 0, 100);
  mapPressure = map(mapSum / samples, 0, 4095, 0, 100);
  lambdaValue = map(lambdaSum / samples, 0, 4095, 50, 150) / 100.0;
  
  // Calculate RPM from cam sensor period
  unsigned long period = 0;
  bool dataAvailable = false;

#ifdef ESP32
  portENTER_CRITICAL(&camMux);
  if (newCamData) {
    period = camPeriod;
    newCamData = false;
    dataAvailable = true;
  }
  portEXIT_CRITICAL(&camMux);
#else
  noInterrupts();
  if (newCamData) {
    period = camPeriod;
    newCamData = false;
    dataAvailable = true;
  }
  interrupts();
#endif

  if (dataAvailable) {
    if (period > 0 && period < 2000000) {
      rpm = (60000000.0 / period) / PULSES_PER_REV;
      rpm = constrain(rpm, 0, 8000);
    } else {
      rpm = 0;
    }
  }
  
  // Timeout detection
  if (micros() - lastCamTime > 2000000) {
    rpm = 0;
    camPeriod = 0; // Reset period as well
  }
}

void measureInjectorPulseWidth() {
  unsigned long pw = 0;
  bool dataAvailable = false;

#ifdef ESP32
  portENTER_CRITICAL(&pulseMux);
  if (newPulseData) {
    pw = measuredPulseWidth;
    newPulseData = false;
    dataAvailable = true;
  }
  portEXIT_CRITICAL(&pulseMux);
#else
  noInterrupts();
  if (newPulseData) {
    pw = measuredPulseWidth;
    newPulseData = false;
    dataAvailable = true;
  }
  interrupts();
#endif

  if (dataAvailable) {
    pulseWidth = pw / 1000.0;
    pulseWidth = constrain(pulseWidth, 0, 20);
    
    // Smooth the display value
    pulseWidthSmooth = pulseWidthSmooth * 0.8 + pulseWidth * 0.2;
  }
}

void controlInjectorGauge() {
  int pwmValue = map(constrain(pulseWidth * 10, 0, 100), 0, 100, 0, 255);
  analogWrite(injectorGaugePin, pwmValue);
}

void handleButton() {
  if (buttonPressed) {
    // Wait to see if it is a long press
    delay(50);

    // Note: digitalRead(buttonPin) == LOW means pressed
    if (digitalRead(buttonPin) == LOW) {
      // Potentially a long press
      unsigned long duration = 0;
      while(digitalRead(buttonPin) == LOW && duration < 3000) {
        delay(10);
        duration += 10;
      }

      if (duration >= 2000) {
        // Very long press: reset lifetime data
        resetLifetimeData();
        drawGui();
      } else if (duration >= 600) {
        // Long press: switch overlay
        currentOverlay = (OverlayMode)((currentOverlay + 1) % OVERLAY_COUNT);
        drawGui();
      } else {
        // Short press: switch visualization mode
        switchVisualizationMode();
      }
    } else {
      // Very short press
      switchVisualizationMode();
    }

    buttonPressed = false;
  }
}

void switchVisualizationMode() {
  currentMode = (VisualizationMode)((currentMode + 1) % MODE_COUNT);
  
  // Clear screen and redraw GUI for new mode
  drawGui();
  
  // If switching to heat map, clear old data
  if (currentMode == MODE_HEAT_MAP || currentMode == MODE_3D_SURFACE || currentMode == MODE_DENSITY_MAP) {
    clearHeatMap();
  }
  
  // Show mode name briefly
  tft.fillRect(80, 100, 160, 40, TFT_NAVY);
  tft.drawRect(80, 100, 160, 40, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_NAVY);
  
  const char* modeName = "";
  switch(currentMode) {
    case MODE_SCATTER_MAP: modeName = "Scatter Map"; break;
    case MODE_HEAT_MAP: modeName = "Heat Map"; break;
    case MODE_3D_SURFACE: modeName = "3D Surface"; break;
    case MODE_AFR_MAP: modeName = "AFR Map"; break;
    case MODE_DENSITY_MAP: modeName = "Density Map"; break;
    default: modeName = "Unknown"; break;
  }
  
  int textWidth = strlen(modeName) * 12;
  tft.setCursor(160 - textWidth/2, 115);
  tft.println(modeName);
  
  delay(1000);
  drawGui();
}

void updateVisualization() {
  switch(currentMode) {
    case MODE_SCATTER_MAP:
      drawScatterMap();
      break;
    case MODE_HEAT_MAP:
      drawHeatMap();
      break;
    case MODE_3D_SURFACE:
      draw3DSurface();
      break;
    case MODE_AFR_MAP:
      drawAFRMap();
      break;
    case MODE_DENSITY_MAP:
      drawDensityMap();
      break;
    default:
      break;
  }
}

void drawScatterMap() {
  if (rpm < 100) return;
  
  int x = map(constrain(rpm, 0, 6000), 0, 6000, 
              CHART_X_OFFSET, CHART_X_OFFSET + CHART_WIDTH);
  
  float load = (mapPressure * throttlePos) / 100.0;
  load = constrain(load, 0, 100);
  
  int y = map(load, 0, 100, 
              CHART_Y_OFFSET + CHART_HEIGHT, CHART_Y_OFFSET);
  
  uint16_t color = getColorForPulseWidth(pulseWidth);
  
  // Draw a 2x2 pixel point
  tft.fillRect(x, y, 2, 2, color);
}

void drawHeatMap() {
  if (rpm < 100) return;
  
  // Calculate cell indices
  int col = constrain((int)(rpm / 500), 0, HEATMAP_COLS - 1); // 500 RPM per cell
  float load = (mapPressure * throttlePos) / 100.0;
  int row = constrain((int)(load / 11.11), 0, HEATMAP_ROWS - 1); // ~11.11% load per cell
  
  // Update cell data with Exponential Moving Average (EMA)
  HeatMapCell* cell = &heatMap[col][row];
  if (cell->count == 0) {
    cell->avgPulseWidth = pulseWidth;
  } else {
    // alpha = 0.1 for slow moving average, reactive but stable
    cell->avgPulseWidth = (cell->avgPulseWidth * 0.9f) + (pulseWidth * 0.1f);
  }
  cell->sumPulseWidth += pulseWidth;
  cell->sumLambda += lambdaValue;
  cell->count++;
  cell->lastUpdate = millis();
  
  // Draw all cells
  int cellWidth = CHART_WIDTH / HEATMAP_COLS;
  int cellHeight = CHART_HEIGHT / HEATMAP_ROWS;
  
  for (int c = 0; c < HEATMAP_COLS; c++) {
    for (int r = 0; r < HEATMAP_ROWS; r++) {
      if (heatMap[c][r].count > 0) {
        float valToShow = 0;
        uint16_t color = 0;
        
        switch(currentOverlay) {
          case OVERLAY_NORMAL:
            valToShow = heatMap[c][r].avgPulseWidth;
            color = getColorForPulseWidth(valToShow);
            break;
          case OVERLAY_LIFETIME:
            valToShow = heatMap[c][r].sumPulseWidth / heatMap[c][r].count;
            color = getColorForPulseWidth(valToShow);
            break;
          case OVERLAY_DIFF:
            {
              // Show deviation: EMA - Lifetime Average
              // Scale color around middle (Green = 0 diff, Red = Positive, Blue = Negative)
              float lifetimeAvg = heatMap[c][r].sumPulseWidth / heatMap[c][r].count;
              float diff = heatMap[c][r].avgPulseWidth - lifetimeAvg;
              color = getColorForDiff(diff);
            }
            break;
          default: break;
        }

        // Fade old cells (not updated in last 10 seconds)
        // Lifetime mode doesn't fade as much
        unsigned long timeSinceUpdate = millis() - heatMap[c][r].lastUpdate;
        if (timeSinceUpdate > 10000) {
           if (currentOverlay != OVERLAY_LIFETIME) {
             if (timeSinceUpdate > 30000) color = COLOR_BACKGROUND;
             else color = (color >> 1) & 0x7BEF; // Darken by 50%
           } else if (timeSinceUpdate > 60000) {
             color = (color >> 1) & 0x7BEF; // Darken slightly after 1 min
           }
        }
        
        int x = CHART_X_OFFSET + c * cellWidth;
        int y = CHART_Y_OFFSET + (HEATMAP_ROWS - 1 - r) * cellHeight;
        
        tft.fillRect(x + 1, y + 1, cellWidth - 2, cellHeight - 2, color);
      }
    }
  }
}

void saveLifetimeData() {
#if defined(ESP32) || defined(UNIT_TEST)
  Preferences prefs;
  prefs.begin("tuning-map", false);
  prefs.putBytes("heatmap", (const void*)heatMap, sizeof(heatMap));
  prefs.end();
  Serial.println("Lifetime data saved to flash");
#endif
}

void loadLifetimeData() {
#if defined(ESP32) || defined(UNIT_TEST)
  Preferences prefs;
  prefs.begin("tuning-map", true);
  size_t bytesRead = prefs.getBytes("heatmap", (void*)heatMap, sizeof(heatMap));
  prefs.end();

  if (bytesRead == sizeof(heatMap)) {
    Serial.println("Lifetime data loaded from flash");
    // Reset lastUpdate for all cells as they were just "loaded"
    for (int c = 0; c < HEATMAP_COLS; c++) {
      for (int r = 0; r < HEATMAP_ROWS; r++) {
        if (heatMap[c][r].count > 0) {
          heatMap[c][r].lastUpdate = millis();
        }
      }
    }
  } else {
    Serial.println("No valid lifetime data found in flash");
  }
#endif
}

void resetLifetimeData() {
#if defined(ESP32) || defined(UNIT_TEST)
  Preferences prefs;
  prefs.begin("tuning-map", false);
  prefs.clear();
  prefs.end();
#endif
  clearHeatMap();
  Serial.println("Lifetime data reset");
}

void draw3DSurface() {
  if (rpm < 100) return;
  
  // Update heat map data with EMA
  int col = constrain((int)(rpm / 500), 0, HEATMAP_COLS - 1);
  float load = (mapPressure * throttlePos) / 100.0;
  int row = constrain((int)(load / 11.11), 0, HEATMAP_ROWS - 1);
  
  HeatMapCell* cell = &heatMap[col][row];
  if (cell->count == 0) {
    cell->avgPulseWidth = pulseWidth;
  } else {
    cell->avgPulseWidth = (cell->avgPulseWidth * 0.9f) + (pulseWidth * 0.1f);
  }
  cell->count++;
  cell->lastUpdate = millis();
  
  // To avoid artifacts, we should clear the chart area before redrawing 3D surface
  // However, full clear might flicker. For now, we overwrite.

  int cellWidth = CHART_WIDTH / HEATMAP_COLS;
  int cellHeight = CHART_HEIGHT / HEATMAP_ROWS;
  
  for (int c = 0; c < HEATMAP_COLS; c++) {
    for (int r = 0; r < HEATMAP_ROWS; r++) {
      if (heatMap[c][r].count > 0) {
        float avgPW = heatMap[c][r].avgPulseWidth;
        
        // Calculate 3D effect
        int baseX = CHART_X_OFFSET + c * cellWidth;
        int baseY = CHART_Y_OFFSET + (HEATMAP_ROWS - 1 - r) * cellHeight;
        
        // Height based on pulse width (up to 20ms)
        int height = map(constrain(avgPW, 0, 20) * 100, 0, 2000, 0, 25);
        
        uint16_t color = getColorForPulseWidth(avgPW);
        
        // Clear previous height if needed (simplified: redraw base and then offset top)
        // tft.fillRect(baseX + 1, baseY + 1, cellWidth - 2, cellHeight - 2, COLOR_BACKGROUND);

        // Draw top face
        tft.fillRect(baseX + 1, baseY - height + 1, cellWidth - 2, cellHeight - 2, color);
        
        // Draw side face (darker)
        uint16_t sideColor = (color >> 1) & 0x7BEF;
        if (height > 2) {
          tft.fillRect(baseX + cellWidth - 2, baseY - height + cellHeight/2, 2, height, sideColor);
        }
      }
    }
  }
}

void drawAFRMap() {
  if (rpm < 100) return;
  
  int x = map(constrain(rpm, 0, 6000), 0, 6000, 
              CHART_X_OFFSET, CHART_X_OFFSET + CHART_WIDTH);
  
  float load = (mapPressure * throttlePos) / 100.0;
  load = constrain(load, 0, 100);
  
  int y = map(load, 0, 100, 
              CHART_Y_OFFSET + CHART_HEIGHT, CHART_Y_OFFSET);
  
  // Color based on lambda/AFR
  uint16_t color = getColorForAFR(lambdaValue);
  
  // Draw larger dot for AFR visualization
  tft.fillCircle(x, y, 2, color);
}

void drawDensityMap() {
  if (rpm < 100) return;
  
  // Update heat map for density counting
  int col = constrain((int)(rpm / 500), 0, HEATMAP_COLS - 1);
  float load = (mapPressure * throttlePos) / 100.0;
  int row = constrain((int)(load / 11.11), 0, HEATMAP_ROWS - 1);
  
  heatMap[col][row].count++;
  heatMap[col][row].lastUpdate = millis();
  
  // Find max count for scaling
  int maxCount = 1;
  for (int c = 0; c < HEATMAP_COLS; c++) {
    for (int r = 0; r < HEATMAP_ROWS; r++) {
      if (heatMap[c][r].count > maxCount) {
        maxCount = heatMap[c][r].count;
      }
    }
  }
  
  // Draw density visualization
  int cellWidth = CHART_WIDTH / HEATMAP_COLS;
  int cellHeight = CHART_HEIGHT / HEATMAP_ROWS;
  
  for (int c = 0; c < HEATMAP_COLS; c++) {
    for (int r = 0; r < HEATMAP_ROWS; r++) {
      int count = heatMap[c][r].count;
      if (count > 0) {
        // Color intensity based on density
        float density = (float)count / maxCount;
        uint8_t intensity = (uint8_t)(density * 255);
        uint16_t color = tft.color565(intensity, intensity, 0); // Yellow gradient
        
        int x = CHART_X_OFFSET + c * cellWidth;
        int y = CHART_Y_OFFSET + (HEATMAP_ROWS - 1 - r) * cellHeight;
        
        tft.fillRect(x + 1, y + 1, cellWidth - 2, cellHeight - 2, color);
      }
    }
  }
}

void drawCurrentPulseWidth() {
  tft.fillRect(DISPLAY_WIDTH - 80, 5, 75, 25, COLOR_BACKGROUND);
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, COLOR_BACKGROUND);
  tft.setCursor(DISPLAY_WIDTH - 78, 5);
  tft.printf("%.2f", pulseWidthSmooth);
  tft.setTextSize(1);
  tft.println("ms");
}

void drawSensorReadings() {
  int yPos = DISPLAY_HEIGHT - 20;
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN, COLOR_BACKGROUND);
  
  tft.fillRect(0, yPos, DISPLAY_WIDTH, 20, COLOR_BACKGROUND);
  
  char buffer[80];
  sprintf(buffer, "RPM:%4.0f TPS:%3.0f%% MAP:%3.0f Lambda:%.2f", 
          rpm, throttlePos, mapPressure, lambdaValue);
  tft.drawString(buffer, 5, yPos + 2);
}

void drawModeIndicator() {
  tft.fillRect(DISPLAY_WIDTH - 80, 18, 75, 20, COLOR_BACKGROUND);
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, COLOR_BACKGROUND);
  tft.setCursor(DISPLAY_WIDTH - 78, 18);
  
  switch(currentMode) {
    case MODE_SCATTER_MAP: tft.print("Scatter"); break;
    case MODE_HEAT_MAP: tft.print("HeatMap"); break;
    case MODE_3D_SURFACE: tft.print("3D Surf"); break;
    case MODE_AFR_MAP: tft.print("AFR Map"); break;
    case MODE_DENSITY_MAP: tft.print("Density"); break;
    default: tft.print("Unknown"); break;
  }

  tft.setCursor(DISPLAY_WIDTH - 78, 28);
  tft.setTextColor(TFT_YELLOW, COLOR_BACKGROUND);
  switch(currentOverlay) {
    case OVERLAY_NORMAL: tft.print("EMA"); break;
    case OVERLAY_LIFETIME: tft.print("Lifetime"); break;
    case OVERLAY_DIFF: tft.print("Diff"); break;
    default: break;
  }
}

void clearHeatMap() {
  for (int c = 0; c < HEATMAP_COLS; c++) {
    for (int r = 0; r < HEATMAP_ROWS; r++) {
      heatMap[c][r].sumPulseWidth = 0;
      heatMap[c][r].sumLambda = 0;
      heatMap[c][r].avgPulseWidth = 0;
      heatMap[c][r].count = 0;
      heatMap[c][r].lastUpdate = 0;
    }
  }
}

uint16_t getColorForPulseWidth(float pw) {
  // Use 15ms as full scale for color
  pw = constrain(pw, 0, 15);
  float normalized = pw / 15.0;
  
  uint8_t r, g, b;
  
  if (normalized < 0.25) {
    float t = normalized / 0.25;
    r = 0;
    g = (uint8_t)(t * 255);
    b = 255;
  } else if (normalized < 0.5) {
    float t = (normalized - 0.25) / 0.25;
    r = 0;
    g = 255;
    b = (uint8_t)(255 * (1 - t));
  } else if (normalized < 0.75) {
    float t = (normalized - 0.5) / 0.25;
    r = (uint8_t)(t * 255);
    g = 255;
    b = 0;
  } else {
    float t = (normalized - 0.75) / 0.25;
    r = 255;
    g = (uint8_t)(255 * (1 - t));
    b = 0;
  }
  
  return tft.color565(r, g, b);
}

uint16_t getColorForDiff(float diff) {
  // diff is EMA - LifetimeAvg
  // Range: -2ms to +2ms
  float range = 2.0;
  float normalized = constrain(diff, -range, range) / range; // -1.0 to 1.0

  if (normalized < 0) {
    // Negative diff (Lower than avg) -> Blue gradient
    return interpolateColor(TFT_GREEN, TFT_BLUE, -normalized);
  } else {
    // Positive diff (Higher than avg) -> Red gradient
    return interpolateColor(TFT_GREEN, TFT_RED, normalized);
  }
}

uint16_t getColorForAFR(float lambda) {
  // Stoichiometric (lambda = 1.0) = Green
  // Rich (lambda < 1.0) = Red gradient
  // Lean (lambda > 1.0) = Blue gradient
  
  if (lambda < 0.9) {
    // Very rich - Red
    return TFT_RED;
  } else if (lambda < 1.0) {
    // Slightly rich - Orange to Yellow
    float t = (lambda - 0.9) / 0.1;
    return interpolateColor(TFT_RED, TFT_YELLOW, t);
  } else if (lambda < 1.1) {
    // Stoichiometric - Green
    return TFT_GREEN;
  } else if (lambda < 1.2) {
    // Slightly lean - Cyan
    float t = (lambda - 1.1) / 0.1;
    return interpolateColor(TFT_GREEN, TFT_CYAN, t);
  } else {
    // Very lean - Blue
    return TFT_BLUE;
  }
}

uint16_t interpolateColor(uint16_t color1, uint16_t color2, float t) {
  uint8_t r1 = (color1 >> 11) << 3;
  uint8_t g1 = ((color1 >> 5) & 0x3F) << 2;
  uint8_t b1 = (color1 & 0x1F) << 3;
  
  uint8_t r2 = (color2 >> 11) << 3;
  uint8_t g2 = ((color2 >> 5) & 0x3F) << 2;
  uint8_t b2 = (color2 & 0x1F) << 3;
  
  uint8_t r = r1 + (r2 - r1) * t;
  uint8_t g = g1 + (g2 - g1) * t;
  uint8_t b = b1 + (b2 - b1) * t;
  
  return tft.color565(r, g, b);
}

// Interrupt service routines
void IRAM_ATTR camISR() {
  unsigned long currentTime = micros();
  
#ifdef ESP32
  portENTER_CRITICAL_ISR(&camMux);
#endif
  if (lastCamTime > 0) {
    camPeriod = currentTime - lastCamTime;
    newCamData = true;
  }
  lastCamTime = currentTime;
#ifdef ESP32
  portEXIT_CRITICAL_ISR(&camMux);
#endif
}

void IRAM_ATTR injectorISR() {
  unsigned long currentTime = micros();
  
#ifdef ESP32
  portENTER_CRITICAL_ISR(&pulseMux);
#endif
  if (digitalRead(injectorPin) == HIGH) {
    injectorOnTime = currentTime;
  } else {
    injectorOffTime = currentTime;
    if (injectorOffTime > injectorOnTime) {
      measuredPulseWidth = injectorOffTime - injectorOnTime;
      newPulseData = true;
    }
  }
#ifdef ESP32
  portEXIT_CRITICAL_ISR(&pulseMux);
#endif
}

void IRAM_ATTR buttonISR() {
  unsigned long currentTime = millis();
  
  // Debounce: ignore if pressed within last 300ms
  if (currentTime - lastButtonPress > 300) {
    buttonPressed = true;
    lastButtonPress = currentTime;
  }
}
