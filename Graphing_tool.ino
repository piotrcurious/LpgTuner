//Created by Bing AI
// This is simple map plotting tool to aid in emgine tuning
// it assumes fairly low resolution display
// it plots injection times on a rpm/load map
// varying color of the dot depending on injection time
// theoretically it will work on Arduino uno
// but esp32 or another fast uC is recommended 

// Include the bluedisplay library
#include "BlueDisplay.hpp"

// Define some pins for the sensors and the injector
int throttlePin = A0; // Throttle position sensor pin
int mapPin = A1; // MAP sensor pin
int lambdaPin = A2; // Lambda probe pin
int camPin = 2; // Cam sensor pin
int injectorPin = 3; // Injector input pin
int injectorGaugePin = 9 ; // Injector timing analog gauge
    // change to PWM capable pin in case of esp32

// Define some variables for the sensors and the injector
float throttle; // Throttle position in percentage
float map; // MAP in kPa
float lambda; // Lambda value
float rpm; // RPM in revolutions per minute
float pulseWidth; // Pulse width in milliseconds
float pulseWidthGfx; // Pulse width for gfx routine 

// Define some variables for the cam sensor interrupt
volatile unsigned long lastCamTime = 0; // Last time the cam sensor triggered in microseconds
volatile unsigned long camPeriod = 0; // Cam sensor period in microseconds

// Define some variables for the display
int displayWidth; // Display width in pixels
int displayHeight; // Display height in pixels
int chartWidth; // Chart width in pixels
int chartHeight; // Chart height in pixels
int chartXOffset; // Chart x offset in pixels
int chartYOffset; // Chart y offset in pixels

// Define some constants for the colors
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F

// Declare callback handler for (re)connect and resize
void initDisplay(void);
void drawGui(void);

void setup()
{
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  
  pinMode(throttlePin, INPUT); // Set the throttle position sensor pin as an input
  pinMode(mapPin, INPUT); // Set the MAP sensor pin as an input
  pinMode(lambdaPin, INPUT); // Set the lambda probe pin as an input
  pinMode(camPin, INPUT_PULLUP); // Set the cam sensor pin as an input with a pull-up resistor
  pinMode(injectorPin, INPUT); // Set the injector pin as an input
  pinMode(injectorGaugePin, OUTPUT); // Set the injector gauge pin as an output

  attachInterrupt(digitalPinToInterrupt(camPin), camISR, RISING); // Attach an interrupt to the cam sensor pin on the rising edge
  attachInterrupt(digitalPinToInterrupt(injectorPin), injectorISR, CHANGE); // Attach an interrupt to the injector pin on both edges
 
  BlueDisplay1.initCommunication(&initDisplay, &drawGui); // Initialize communication with BlueDisplay app
  
}

void loop()
{
  checkAndHandleEvents(); // Check and handle events from BlueDisplay app
  
  readSensors(); // Read the sensors and store the measurements
  
  measureInjectorPulseWidth(); // Measure the actual pulse width of the injector using an interrupt
  
  controlInjector(); // Control the injector using PWM
  
  drawChart(); // Draw a chart of injection times in function of engine rpm and load using pixels with color changing with injection time
  
  drawCurrentPulseWidth(); // Draw current injection time on top right corner
  
}

void initDisplay(void)
{
  displayWidth = BlueDisplay1.getDisplayWidth(); // Get display width in pixels from BlueDisplay app
  displayHeight = BlueDisplay1.getDisplayHeight(); // Get display height in pixels from BlueDisplay app
  
  chartWidth = displayWidth - 20; // Set chart width to leave some margin on both sides
  chartHeight = displayHeight - 40; // Set chart height to leave some margin on top and bottom
  chartXOffset = 10; // Set chart x offset to center the chart horizontally
  chartYOffset = 20; // Set chart y offset to center the chart vertically
  
}

void drawGui(void)
{
  BlueDisplay1.clearDisplay(COLOR_WHITE); // Clear display with white color
  
  BlueDisplay1.drawText(10, 0, "Injector Pulse Width Visualization", TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE); // Draw title text
  
  BlueDisplay1.drawLine(chartXOffset, chartYOffset, chartXOffset + chartWidth, chartYOffset, COLOR_BLACK); // Draw x axis
  BlueDisplay1.drawLine(chartXOffset, chartYOffset + chartHeight, chartXOffset + chartWidth, chartYOffset + chartHeight, COLOR_BLACK); // Draw x axis
  BlueDisplay1.drawLine(chartXOffset, chartYOffset, chartXOffset, chartYOffset + chartHeight, COLOR_BLACK); // Draw y axis
  BlueDisplay1.drawLine(chartXOffset + chartWidth, chartYOffset, chartXOffset + chartWidth, chartYOffset + chartHeight, COLOR_BLACK); // Draw y axis
  
  BlueDisplay1.drawText(chartXOffset + chartWidth / 2 - 20, displayHeight - 10, "RPM", TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE); // Draw x label
  BlueDisplay1.drawText(0, displayHeight / 2 - 10, "Load", TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE); // Draw y label
  
}

void readSensors()
{
  // Read the throttle position sensor and map it to a percentage value between 0 and 100
  throttle = map(analogRead(throttlePin), 0, 1023, 0, 100);
  
  // Read the MAP sensor and map it to a kPa value between 0 and 100
  map = map(analogRead(mapPin), 0, 1023, 0, 100);
  
  // Read the lambda probe and map it to a ratio value between 0.5 and 1.5
  lambda = map(analogRead(lambdaPin), 0, 1023, 0.5, 1.5);
  
  // Read the cam sensor using an interrupt and calculate the RPM based on the cam period
  rpm = 60000000 / camPeriod; // Calculate RPM in revolutions per minute based on the cam period in microseconds
}

void measureInjectorPulseWidth()
{
  pulseWidthGfx = PulseWidth / 1000.0; // Calculate the pulse width in milliseconds based on the injector on and off times in microseconds
}

void controlInjectorGauge()
{
  // Control the injector gauge using PWM with a frequency of 100 Hz and a duty cycle proportional to the pulse width
  analogWrite(injectorGaugePin, map(pulseWidth, 0, 10, 0, 255)); // Map the pulse width to a value between 0 and 255 and write it to the injector pin using PWM
}

void drawChart()
{
  // Draw a chart of injection times in function of engine rpm and load using pixels with color changing with injection time
  
  // Map the rpm to a pixel x coordinate between chartXOffset and chartXOffset + chartWidth
  int x = map(rpm, 0, 6000, chartXOffset, chartXOffset + chartWidth);
  
  // Map the load to a pixel y coordinate between chartYOffset and chartYOffset + chartHeight
  int y = map(map * throttle / 100, 0, 100, chartYOffset + chartHeight, chartYOffset);
  
  // Map the pulse width to a color value between blue and red
  int color = map(pulseWidth, 0, 10, COLOR_BLUE, COLOR_RED);
  
  // Draw a pixel at (x, y) with the color value
  BlueDisplay1.drawPixel(x, y, color);
}

void drawCurrentPulseWidth()
{
  // Draw current injection time on top right corner
  
  // Convert the pulse width to a string with two decimal places
  char pwString[10];
  sprintf(pwString, "%.2f ms", pulseWidth);
  
  // Draw a white rectangle to clear the previous text
  BlueDisplay1.fillRect(displayWidth - 60, 0, displayWidth, 20, COLOR_WHITE);
  
  // Draw the pulse width string with black color and text size 11
  BlueDisplay1.drawText(displayWidth - 60, 0, pwString, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}

// Define a function to handle the cam sensor interrupt
void camISR()
{
  unsigned long currentCamTime = micros(); // Get the current time in microseconds
  
  if (lastCamTime > 0) // If this is not the first interrupt
  {
    camPeriod = currentCamTime - lastCamTime; // Calculate the cam period as the difference between the current and last time
  }
  
  lastCamTime = currentCamTime; // Update the last time to the current time
}

// Define a function to handle the injector interrupt
void injectorISR()
{
  unsigned long currentInjectorTime = micros(); // Get the current time in microseconds
  
  if (digitalRead(injectorPin) == HIGH) // If the injector pin is high
  {
    injectorOnTime = currentInjectorTime; // Update the injector on time to the current time
  }
  else // If the injector pin is low
  {
    injectorOffTime = currentInjectorTime; // Update the injector off time to the current time
    pulseWidth = (injectorOffTime - injectorOnTime)
  }

}
