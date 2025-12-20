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

//--------- INPUTS section

#include "MAX6675.h"
const int selectPin = 5; // CS pin for MAX6675 (adjust as needed)

MAX6675 thermoCouple(selectPin);
uint32_t last_conversion_time = 0; 
#define MAX6675_CONVERSION_RATE 200 // minimum 220ms for MAX6675

// ADC pin (ESP32 has multiple ADC pins)
const int analogInPin = 34; // Use GPIO34 (ADC1_CH6)
#define MAX_ANALOG_VOLTAGE_A0 3.3
#define ADC_RESOLUTION_A0 4096 // ESP32 has 12-bit ADC

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

// Define the layout array - scaled for 320x240
WidgetElement layouts[][5] = {
  // Layout 1: temperature, lambda sensor
  {
    // Widget 1: Temperature decimal
    {
      "Temp01",
      1, // MAX6675 thermocouple
      "Temp:",
      10,
      10,
      2, // Font size 2
      500,
      1, // Decimal display
      0.0,
      1023.0,
      {0, 1023, WHITE, 120, 20, 0, 0, 0, 0, 0}
    },
    // Widget 2: Lambda sensor
    {
      "lambda",
      2, // Analog input
      "Lambda:",
      170,
      10,
      2,
      1000,
      1, // Decimal display
      0.0,
      3.3,
      {0, 1023, CYAN, 120, 20, 0, 0, 0, 0, 0}
    },
    // Widget 3: Temperature bar with fade
    {
      "Temp01",
      1,
      "",
      10,
      50,
      1,
      500,
      0x21, // Fading bar
      20.0,
      800.0,
      {20, 800, ORANGE, 300, 8, 20, 50, 0, 0, 0}
    },
    // Widget 4: Temperature rolling graph
    {
      "Temp01",
      1,
      "",
      10,
      80,
      1,
      500,
      0x30, // Rolling graph buffer 0
      30.0,
      800.0,
      {30, 800, RED, 140, 80, 0, 0, 0, 0, 0}
    },
    // Widget 5: Lambda analog gauge
    {
      "lambda",
      2,
      "",
      170,
      80,
      1,
      50,
      4, // Analog gauge
      0.0,
      3.3,
      {0, 3.3, GREEN, 60, 60, 40, 0, 0, 0, 0}
    }
  },
  // Layout 2: Random values demo
  {
    // Widget 1: Voltage
    {
      "volt",
      0, // Random
      "Volt:",
      10,
      10,
      2,
      1000,
      1,
      0,
      1023,
      {0, 1023, YELLOW, 120, 20, 0, 0, 0, 0, 0}
    },
    // Widget 2: Current
    {
      "curr",
      0,
      "Curr:",
      170,
      10,
      2,
      1000,
      1,
      0,
      1023,
      {0, 1023, MAGENTA, 120, 20, 0, 0, 0, 0, 0}
    },
    // Widget 3: Power bar
    {
      "pow",
      0,
      "Power:",
      10,
      50,
      1,
      500,
      2, // Bar
      0,
      1023,
      {0, 1023, BLUE, 140, 30, 0, 0, 0, 0, 0}
    },
    // Widget 4: Energy rolling graph
    {
      "en",
      0,
      "Energy:",
      170,
      50,
      1,
      100,
      0x31, // Rolling graph buffer 1
      0,
      1023,
      {0, 1023, CYAN, 140, 80, 0, 0, 0, 0, 0}
    },
    // Widget 5: Frequency gauge
    {
      "freq",
      0,
      "Freq:",
      10,
      150,
      1,
      500,
      4, // Analog gauge
      0,
      100,
      {0, 100, GREEN, 80, 80, 50, 0, 0, 0, 0}
    }
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
unsigned long layoutChangeInterval = 16000;

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initDisplay() {
  tft.init();
  tft.setRotation(1); // Landscape mode (320x240)
  tft.fillScreen(BLACK);
  
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(80, 110);
  tft.print("Lambda Angel");
  
  delay(1000);
  tft.fillScreen(BLACK);
}

void updateDisplay() {
  WidgetElement* layout = layouts[currentLayout];

  for (int i = 0; i < NUM_WIDGETS; i++) {
    WidgetElement widget = layout[i];

    if (millis() - lastUpdate[i] >= widget.rate) {
      lastUpdate[i] = millis();
      currentValue[currentLayout][i] = readWidget(widget.source);

      switch (widget.mode) {
        case 0: // Hex
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);              
          tft.print("0x");
          tft.print((int)currentValue[currentLayout][i], HEX);
          break;
          
        case 1: // Decimal
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);
          tft.print(currentValue[currentLayout][i], 2);
          break;
          
        case 0x2: // Bar
          clearArea(widget);
          displayBar(widget, currentValue[currentLayout][i]);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          break;

        case 0x21: // Bar with fade
          clearAreaRate(widget, widget.params[6]);
          displayBarRate(widget, currentValue[currentLayout][i]);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          break;
          
        case 0x30: // Rolling graph buffer 0
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          displayGraphx30(widget, currentValue[currentLayout][i]);
          break;

        case 0x31: // Rolling graph buffer 1
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          displayGraphx31(widget, currentValue[currentLayout][i]);
          break;

        case 4: // Analog gauge
          clearArea(widget);
          tft.setTextSize(widget.font);
          tft.setTextColor(widget.params[2]);
          tft.setCursor(widget.x, widget.y);
          tft.print(widget.label);          
          displayGauge(widget, currentValue[currentLayout][i]);
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
        float temp = thermoCouple.getTemperature();
        return temp;
      }
      
    case 2: // Analog input
      {
        float voltage = analogRead(analogInPin) * (MAX_ANALOG_VOLTAGE_A0 / ADC_RESOLUTION_A0);
        return voltage;
      }
      
    default: // Random values
      return random(0, 1024);
  }
}

void clearArea(WidgetElement widget) {
  int width = widget.params[3];
  int height = widget.params[4];
  tft.fillRect(widget.x, widget.y, width, height + 1, BLACK);
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
  fillRectWithRate(widget.x, widget.y, bar, height, color, rate);
}

void displayGraphx30(WidgetElement widget, float value) {
  float min = widget.value_min;
  float max = widget.value_max;
  int color = widget.params[2];
  int width = widget.params[3];
  int height = widget.params[4];

  int graph = fmap(value, min, max, 0, height);

  // Shift buffer left
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
}; i < BUFFER0_SIZE - 1; i++) {
    buffer0[i] = buffer0[i + 1];
  }
  buffer0[BUFFER0_SIZE - 1] = graph;

  // Draw graph
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

  // Shift buffer left
  for (uint16_t i = 0; i < BUFFER1_SIZE - 1; i++) {
    buffer1[i] = buffer1[i + 1];
  }
  buffer1[BUFFER1_SIZE - 1] = graph;

  // Draw graph
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

  // Draw gauge arc
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
  thermoCouple.setSPIspeed(4000000); // 4 MHz
  
  // Configure ADC
  analogReadResolution(12); // 12-bit resolution
  analogSetAttenuation(ADC_11db); // 0-3.3V range
}

void switch_layout() {
  if (millis() - lastLayoutChange >= layoutChangeInterval) {
    lastLayoutChange = millis();
    currentLayout++;
    
    if (currentLayout >= NUM_LAYOUTS) {
      currentLayout = 0;
    }
    
    tft.fillScreen(BLACK);
  }
}

void loop() {
  // Uncomment to enable automatic layout switching
  // switch_layout();
  
  updateDisplay();
}
