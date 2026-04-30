// Engine Tuning Map Plotting Tool
// Displays injection timing on RPM/Load map with multiple visualization modes
// Optimized for ESP32 with 320x240 TFT_eSPI display
// Button on GPIO 0 (BOOT button) switches visualization modes

#ifdef UNIT_TEST
#include "mock_tft.h"
#include "mock_arduino.h"
#else
#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>
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
LGFX tft;
LGFX_Sprite pwSprite(&tft);
LGFX_Sprite sensorSprite(&tft);

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
unsigned long modeNameShowTime = 0;
const char* currentModeName = "";

// Color definitions
#define COLOR_BACKGROUND 0x0000 // Black
#define COLOR_TEXT 0xFFFF       // White
#define COLOR_AXIS 0xFFFF       // White
#define COLOR_GRID 0x7BEF       // Dark Grey

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
bool measureInjectorPulseWidth();
void updateStatisticalMaps();
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
void drawColorLegend();
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

  // Initialize Sprites
  pwSprite.createSprite(80, 25);
  sensorSprite.createSprite(DISPLAY_WIDTH, 20);

  tft.startWrite();
  drawGui();
  tft.endWrite();
  
  Serial.println("Initialization complete");
  Serial.println("Press BOOT button to switch visualization modes");
}

void loop() {
  readSensors();
  bool hasNewPulse = measureInjectorPulseWidth();

  // Update statistical maps only when new pulse data arrives
  if (hasNewPulse && rpm >= 100) {
    updateStatisticalMaps();
  }

  controlInjectorGauge();
  
  // Handle mode switching
  handleButton();
  
  // Update display at controlled intervals
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = millis();
    tft.startWrite();
    updateVisualization();
    drawCurrentPulseWidth();
    drawSensorReadings();
    drawModeIndicator();

    // Draw mode name overlay if active
    if (millis() < modeNameShowTime) {
      tft.fillRect(80, 100, 160, 40, 0x000F); // Navy
      tft.drawRect(80, 100, 160, 40, 0xFFFF); // White
      tft.setTextSize(2);
      tft.setTextColor(0xFFE0, 0x000F); // Yellow, Navy
      int textWidth = strlen(currentModeName) * 12;
      tft.setCursor(160 - textWidth/2, 115);
      tft.println(currentModeName);
    }

    tft.endWrite();
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
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
}

void drawGui() {
  tft.fillScreen(COLOR_BACKGROUND);
  initDisplay();
  
  // Draw title
  tft.setTextSize(1);
  tft.setTextColor(0x07FF, COLOR_BACKGROUND); // Cyan
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

  drawColorLegend();
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

bool measureInjectorPulseWidth() {
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
  return dataAvailable;
}

void controlInjectorGauge() {
  int pwmValue = map(constrain(pulseWidth * 10, 0, 100), 0, 100, 0, 255);
  analogWrite(injectorGaugePin, pwmValue);
}

enum ButtonState {
  BTN_IDLE,
  BTN_PRESSED,
  BTN_WAIT_RELEASE
};

ButtonState btnState = BTN_IDLE;
unsigned long btnPressTime = 0;

void handleButton() {
  bool isPressed = (digitalRead(buttonPin) == LOW);
  unsigned long now = millis();

  switch(btnState) {
    case BTN_IDLE:
      if (isPressed) {
        btnPressTime = now;
        btnState = BTN_PRESSED;
        buttonPressed = false; // Consume ISR flag
      }
      break;

    case BTN_PRESSED:
      if (!isPressed) {
        // Released - determine if it was a short or long press
        unsigned long duration = now - btnPressTime;
        if (duration >= 50) { // Debounce
           if (duration >= 2000) {
             resetLifetimeData();
             tft.startWrite();
             drawGui();
             tft.endWrite();
           } else if (duration >= 600) {
             currentOverlay = (OverlayMode)((currentOverlay + 1) % OVERLAY_COUNT);
             tft.startWrite();
             drawGui();
             tft.endWrite();
           } else {
             switchVisualizationMode();
           }
        }
        btnState = BTN_IDLE;
      } else {
        // Still pressed - could provide visual feedback for very long press
        if (now - btnPressTime >= 2000) {
          // Could show a "Resetting..." message here if desired
        }
      }
      break;

    case BTN_WAIT_RELEASE:
      if (!isPressed) btnState = BTN_IDLE;
      break;
  }

  // Also handle the legacy ISR flag for compatibility or safety
  if (buttonPressed && btnState == BTN_IDLE) {
    // If ISR triggered but we missed it in IDLE
    btnPressTime = now;
    btnState = BTN_PRESSED;
    buttonPressed = false;
  }
}

void switchVisualizationMode() {
  currentMode = (VisualizationMode)((currentMode + 1) % MODE_COUNT);
  
  switch(currentMode) {
    case MODE_SCATTER_MAP: currentModeName = "Scatter Map"; break;
    case MODE_HEAT_MAP: currentModeName = "Heat Map"; break;
    case MODE_3D_SURFACE: currentModeName = "3D Surface"; break;
    case MODE_AFR_MAP: currentModeName = "AFR Map"; break;
    case MODE_DENSITY_MAP: currentModeName = "Density Map"; break;
    default: currentModeName = "Unknown"; break;
  }

  modeNameShowTime = millis() + 1500; // Show for 1.5 seconds

  tft.startWrite();
  drawGui();
  tft.endWrite();
}

void updateStatisticalMaps() {
  // Calculate cell indices
  int col = (int)(rpm / 500);
  float load = (mapPressure * throttlePos) / 100.0;
  int row = (int)(load / 11.11);

  // Bounds checking
  if (col < 0 || col >= HEATMAP_COLS || row < 0 || row >= HEATMAP_ROWS) return;

  HeatMapCell* cell = &heatMap[col][row];

  // Update EMA
  if (cell->count == 0) {
    cell->avgPulseWidth = pulseWidth;
  } else {
    // alpha = 0.1 for slow moving average, reactive but stable
    cell->avgPulseWidth = (cell->avgPulseWidth * 0.9f) + (pulseWidth * 0.1f);
  }

  // Update Lifetime Average
  cell->sumPulseWidth += pulseWidth;
  cell->sumLambda += lambdaValue;
  cell->count++;
  cell->lastUpdate = millis();
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

void drawColorLegend() {
  const int LEGEND_X = CHART_X_OFFSET + CHART_WIDTH + 5;
  const int LEGEND_Y = CHART_Y_OFFSET;
  const int LEGEND_W = 10;
  const int LEGEND_H = CHART_HEIGHT;

  if (currentOverlay == OVERLAY_DIFF) {
    // Diff mode legend (-2ms to +2ms)
    for (int i = 0; i < LEGEND_H; i++) {
      float diff = 2.0f - (4.0f * i / LEGEND_H);
      uint16_t color = getColorForDiff(diff);
      tft.drawFastHLine(LEGEND_X, LEGEND_Y + i, LEGEND_W, color);
    }
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(1);
    tft.drawString("+2", LEGEND_X + LEGEND_W + 2, LEGEND_Y);
    tft.drawString(" 0", LEGEND_X + LEGEND_W + 2, LEGEND_Y + LEGEND_H / 2 - 4);
    tft.drawString("-2", LEGEND_X + LEGEND_W + 2, LEGEND_Y + LEGEND_H - 8);
  } else if (currentMode == MODE_AFR_MAP) {
    // AFR/Lambda legend (0.9 to 1.2)
    for (int i = 0; i < LEGEND_H; i++) {
      float lambda = 0.85f + (0.4f * i / LEGEND_H); // Backwards to match bottom-up
      uint16_t color = getColorForAFR(lambda);
      tft.drawFastHLine(LEGEND_X, LEGEND_Y + (LEGEND_H - 1 - i), LEGEND_W, color);
    }
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(1);
    tft.drawString("1.25", LEGEND_X + LEGEND_W + 2, LEGEND_Y);
    tft.drawString("1.00", LEGEND_X + LEGEND_W + 2, LEGEND_Y + LEGEND_H / 2 - 4);
    tft.drawString("0.85", LEGEND_X + LEGEND_W + 2, LEGEND_Y + LEGEND_H - 8);
  } else {
    // Standard PW legend (0ms to 15ms)
    for (int i = 0; i < LEGEND_H; i++) {
      float pw = 15.0f * (LEGEND_H - 1 - i) / LEGEND_H;
      uint16_t color = getColorForPulseWidth(pw);
      tft.drawFastHLine(LEGEND_X, LEGEND_Y + i, LEGEND_W, color);
    }
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(1);
    tft.drawString("15ms", LEGEND_X + LEGEND_W + 2, LEGEND_Y);
    tft.drawString("0ms", LEGEND_X + LEGEND_W + 2, LEGEND_Y + LEGEND_H - 8);
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
  // Optimized pseudo-3D rendering with back-to-front drawing to handle occlusions
  int cellWidth = CHART_WIDTH / HEATMAP_COLS;
  int cellHeight = CHART_HEIGHT / HEATMAP_ROWS;
  
  // We draw from top-right to bottom-left to minimize artifacts
  for (int r = HEATMAP_ROWS - 1; r >= 0; r--) {
    for (int c = HEATMAP_COLS - 1; c >= 0; c--) {
      if (heatMap[c][r].count > 0) {
        float valToShow = 0;
        uint16_t color = 0;
        
        // Use active overlay for 3D height/color
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
              float lifetimeAvg = heatMap[c][r].sumPulseWidth / heatMap[c][r].count;
              valToShow = heatMap[c][r].avgPulseWidth - lifetimeAvg;
              color = getColorForDiff(valToShow);
              // Shift range for display height (center around middle)
              valToShow += 5.0;
            }
            break;
          default: break;
        }

        // Calculate 3D effect
        int baseX = CHART_X_OFFSET + c * cellWidth;
        int baseY = CHART_Y_OFFSET + (HEATMAP_ROWS - 1 - r) * cellHeight;
        
        // Height based on value
        int height = map(constrain(valToShow, 0, 20) * 100, 0, 2000, 0, 25);
        
        // Draw side face (shading)
        uint16_t sideColor = (color >> 1) & 0x7BEF;
        if (height > 0) {
           tft.fillRect(baseX + 1, baseY - height + 1, cellWidth - 2, height + cellHeight - 2, sideColor);
        }
        
        // Draw top face
        tft.fillRect(baseX + 1, baseY - height + 1, cellWidth - 2, cellHeight - 2, color);
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
  pwSprite.fillScreen(COLOR_BACKGROUND);
  pwSprite.setTextSize(2);
  pwSprite.setTextColor(0xFFE0, COLOR_BACKGROUND); // Yellow
  pwSprite.setCursor(0, 0);
  pwSprite.printf("%.2f", pulseWidthSmooth);
  pwSprite.setTextSize(1);
  pwSprite.println("ms");
  pwSprite.pushSprite(DISPLAY_WIDTH - 80, 5);
}

void drawSensorReadings() {
  sensorSprite.fillScreen(COLOR_BACKGROUND);
  sensorSprite.setTextSize(1);
  sensorSprite.setTextColor(0x07E0, COLOR_BACKGROUND); // Green
  
  char buffer[80];
  sprintf(buffer, "RPM:%4.0f TPS:%3.0f%% MAP:%3.0f Lambda:%.2f", 
          rpm, throttlePos, mapPressure, lambdaValue);
  sensorSprite.drawString(buffer, 5, 2);
  sensorSprite.pushSprite(0, DISPLAY_HEIGHT - 20);
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
    return interpolateColor(0x07E0, 0x001F, -normalized); // Green to Blue
  } else {
    // Positive diff (Higher than avg) -> Red gradient
    return interpolateColor(0x07E0, 0xF800, normalized); // Green to Red
  }
}

uint16_t getColorForAFR(float lambda) {
  // Stoichiometric (lambda = 1.0) = Green
  // Rich (lambda < 1.0) = Red gradient
  // Lean (lambda > 1.0) = Blue gradient
  
  if (lambda < 0.9) {
    // Very rich - Red
    return 0xF800; // Red
  } else if (lambda < 1.0) {
    // Slightly rich - Orange to Yellow
    float t = (lambda - 0.9) / 0.1;
    return interpolateColor(0xF800, 0xFFE0, t); // Red to Yellow
  } else if (lambda < 1.1) {
    // Stoichiometric - Green
    return 0x07E0; // Green
  } else if (lambda < 1.2) {
    // Slightly lean - Cyan
    float t = (lambda - 1.1) / 0.1;
    return interpolateColor(0x07E0, 0x07FF, t); // Green to Cyan
  } else {
    // Very lean - Blue
    return 0x001F; // Blue
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
