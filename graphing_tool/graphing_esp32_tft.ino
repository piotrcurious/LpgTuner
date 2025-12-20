// Engine Tuning Map Plotting Tool
// Displays injection timing on RPM/Load map with color-coded visualization
// Optimized for ESP32 with 320x240 TFT_eSPI display

#include <TFT_eSPI.h>

// Initialize display
TFT_eSPI tft = TFT_eSPI();

// Define pins for sensors and injector
const int throttlePin = 34;      // Throttle position sensor (ADC1)
const int mapPin = 35;           // MAP sensor (ADC1)
const int lambdaPin = 32;        // Lambda probe (ADC1)
const int camPin = 26;           // Cam sensor (interrupt capable)
const int injectorPin = 27;      // Injector input (interrupt capable)
const int injectorGaugePin = 25; // PWM output for gauge (PWM capable)

// Sensor and injector variables
float throttlePos = 0;           // Throttle position in percentage
float mapPressure = 0;           // MAP in kPa
float lambdaValue = 0;           // Lambda value
float rpm = 0;                   // RPM in revolutions per minute
float pulseWidth = 0;            // Pulse width in milliseconds
float pulseWidthSmooth = 0;      // Smoothed pulse width for display

// Cam sensor interrupt variables
volatile unsigned long lastCamTime = 0;
volatile unsigned long camPeriod = 1000000; // Initialize to prevent div by zero
volatile bool newCamData = false;

// Injector interrupt variables
volatile unsigned long injectorOnTime = 0;
volatile unsigned long injectorOffTime = 0;
volatile unsigned long measuredPulseWidth = 0;
volatile bool newPulseData = false;

// Display constants
const int DISPLAY_WIDTH = 320;
const int DISPLAY_HEIGHT = 240;
const int CHART_WIDTH = 280;
const int CHART_HEIGHT = 180;
const int CHART_X_OFFSET = 30;
const int CHART_Y_OFFSET = 35;

// Update intervals
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100; // Update every 100ms

// Color definitions
#define COLOR_BACKGROUND TFT_BLACK
#define COLOR_TEXT TFT_WHITE
#define COLOR_AXIS TFT_WHITE
#define COLOR_GRID TFT_DARKGREY

// Forward declarations
void IRAM_ATTR camISR();
void IRAM_ATTR injectorISR();
void initDisplay();
void drawGui();
void readSensors();
void measureInjectorPulseWidth();
void controlInjectorGauge();
void drawChart();
void drawCurrentPulseWidth();
void drawSensorReadings();
uint16_t getColorForPulseWidth(float pw);

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
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(camPin), camISR, RISING);
  attachInterrupt(digitalPinToInterrupt(injectorPin), injectorISR, CHANGE);
  
  // Initialize display
  tft.init();
  tft.setRotation(1); // Landscape mode
  initDisplay();
  drawGui();
  
  Serial.println("Initialization complete");
}

void loop() {
  readSensors();
  measureInjectorPulseWidth();
  controlInjectorGauge();
  
  // Update display at controlled intervals
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = millis();
    drawChart();
    drawCurrentPulseWidth();
    drawSensorReadings();
  }
  
  // Optional: Print debug info to serial
  static unsigned long lastSerialPrint = 0;
  if (millis() - lastSerialPrint >= 1000) {
    lastSerialPrint = millis();
    Serial.printf("RPM: %.0f, Load: %.1f%%, PW: %.2fms, Lambda: %.2f\n", 
                  rpm, (mapPressure * throttlePos / 100.0), pulseWidth, lambdaValue);
  }
}

void initDisplay() {
  tft.fillScreen(COLOR_BACKGROUND);
  tft.setTextColor(COLOR_TEXT, COLOR_BACKGROUND);
}

void drawGui() {
  // Draw title
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, COLOR_BACKGROUND);
  tft.drawString("Injection Pulse Width Map", 10, 5);
  
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
  
  // Vertical text approximation for "Load"
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
  if (newCamData) {
    noInterrupts();
    unsigned long period = camPeriod;
    newCamData = false;
    interrupts();
    
    if (period > 0 && period < 1000000) { // Valid range check
      rpm = 60000000.0 / period;
      rpm = constrain(rpm, 0, 8000); // Reasonable RPM range
    } else {
      rpm = 0;
    }
  }
  
  // Timeout detection - if no cam signal for 2 seconds, set RPM to 0
  if (micros() - lastCamTime > 2000000) {
    rpm = 0;
  }
}

void measureInjectorPulseWidth() {
  if (newPulseData) {
    noInterrupts();
    unsigned long pw = measuredPulseWidth;
    newPulseData = false;
    interrupts();
    
    pulseWidth = pw / 1000.0; // Convert to milliseconds
    pulseWidth = constrain(pulseWidth, 0, 20); // Reasonable range
    
    // Smooth the display value
    pulseWidthSmooth = pulseWidthSmooth * 0.8 + pulseWidth * 0.2;
  }
}

void controlInjectorGauge() {
  // Output PWM proportional to pulse width (0-10ms range)
  int pwmValue = map(constrain(pulseWidth * 10, 0, 100), 0, 100, 0, 255);
  analogWrite(injectorGaugePin, pwmValue);
}

void drawChart() {
  if (rpm < 100) return; // Don't plot at very low/zero RPM
  
  // Map RPM to x coordinate (0-6000 RPM range)
  int x = map(constrain(rpm, 0, 6000), 0, 6000, 
              CHART_X_OFFSET, CHART_X_OFFSET + CHART_WIDTH);
  
  // Calculate engine load (MAP * Throttle / 100)
  float load = (mapPressure * throttlePos) / 100.0;
  load = constrain(load, 0, 100);
  
  // Map load to y coordinate (inverted - 0 at bottom, 100 at top)
  int y = map(load, 0, 100, 
              CHART_Y_OFFSET + CHART_HEIGHT, CHART_Y_OFFSET);
  
  // Get color based on pulse width
  uint16_t color = getColorForPulseWidth(pulseWidth);
  
  // Draw a 2x2 pixel point for better visibility
  tft.fillRect(x, y, 2, 2, color);
}

void drawCurrentPulseWidth() {
  // Clear area for text
  tft.fillRect(DISPLAY_WIDTH - 80, 5, 75, 25, COLOR_BACKGROUND);
  
  // Draw pulse width value
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, COLOR_BACKGROUND);
  tft.setCursor(DISPLAY_WIDTH - 78, 5);
  tft.printf("%.2f", pulseWidthSmooth);
  tft.setTextSize(1);
  tft.println("ms");
}

void drawSensorReadings() {
  // Display additional sensor data at bottom
  int yPos = DISPLAY_HEIGHT - 20;
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN, COLOR_BACKGROUND);
  
  // Clear area
  tft.fillRect(0, yPos, DISPLAY_WIDTH, 20, COLOR_BACKGROUND);
  
  // Draw readings
  char buffer[80];
  sprintf(buffer, "RPM:%4.0f TPS:%3.0f%% MAP:%3.0f Lambda:%.2f", 
          rpm, throttlePos, mapPressure, lambdaValue);
  tft.drawString(buffer, 5, yPos + 2);
}

uint16_t getColorForPulseWidth(float pw) {
  // Create smooth color gradient from blue (0ms) to red (10ms+)
  // Blue -> Cyan -> Green -> Yellow -> Red
  
  pw = constrain(pw, 0, 10);
  float normalized = pw / 10.0; // 0.0 to 1.0
  
  uint8_t r, g, b;
  
  if (normalized < 0.25) {
    // Blue to Cyan
    float t = normalized / 0.25;
    r = 0;
    g = (uint8_t)(t * 255);
    b = 255;
  } else if (normalized < 0.5) {
    // Cyan to Green
    float t = (normalized - 0.25) / 0.25;
    r = 0;
    g = 255;
    b = (uint8_t)(255 * (1 - t));
  } else if (normalized < 0.75) {
    // Green to Yellow
    float t = (normalized - 0.5) / 0.25;
    r = (uint8_t)(t * 255);
    g = 255;
    b = 0;
  } else {
    // Yellow to Red
    float t = (normalized - 0.75) / 0.25;
    r = 255;
    g = (uint8_t)(255 * (1 - t));
    b = 0;
  }
  
  // Convert RGB888 to RGB565
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Interrupt service routines
void IRAM_ATTR camISR() {
  unsigned long currentTime = micros();
  
  if (lastCamTime > 0) {
    camPeriod = currentTime - lastCamTime;
    newCamData = true;
  }
  
  lastCamTime = currentTime;
}

void IRAM_ATTR injectorISR() {
  unsigned long currentTime = micros();
  
  if (digitalRead(injectorPin) == HIGH) {
    injectorOnTime = currentTime;
  } else {
    injectorOffTime = currentTime;
    if (injectorOffTime > injectorOnTime) {
      measuredPulseWidth = injectorOffTime - injectorOnTime;
      newPulseData = true;
    }
  }
}
