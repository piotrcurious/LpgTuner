// Include the libraries for the OLED display
#include <Adafruit_SH110X.h>
#include <Adafruit_GFX.h>

// Define the pins for the OLED display
#define OLED_RESET 4
#define OLED_CS 10
#define OLED_DC 11
#define OLED_CLK 13
#define OLED_MOSI 12

// Create an OLED object
Adafruit_SH1107 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// Define the pins for the oxygen sensor and the injector signal
#define O2_PIN A0
#define INJ_PIN 2

// Define the threshold voltage for the oxygen sensor
#define O2_THRESHOLD 0.45

// Define some variables for storing the signal values and timings
float o2_voltage = 0; // The voltage of the oxygen sensor
float inj_duty = 0; // The duty cycle of the injector signal
float inj_period = 0; // The period of the injector signal
float afr = 0; // The estimated air-fuel ratio
float corr = 0; // The estimated correction factor

// Define some variables for storing the previous and current states of the signals
bool o2_state = LOW; // The state of the oxygen sensor (LOW = lean, HIGH = rich)
bool o2_prev_state = LOW; // The previous state of the oxygen sensor
bool inj_state = LOW; // The state of the injector signal (LOW = off, HIGH = on)
bool inj_prev_state = LOW; // The previous state of the injector signal

// Define some variables for storing the timestamps of the signal transitions
unsigned long o2_rise_time = 0; // The time when the oxygen sensor rises from LOW to HIGH
unsigned long o2_fall_time = 0; // The time when the oxygen sensor falls from HIGH to LOW
unsigned long inj_rise_time = 0; // The time when the injector signal rises from LOW to HIGH
unsigned long inj_fall_time = 0; // The time when the injector signal falls from HIGH to LOW

// Define some constants for the rolling graph
#define GRAPH_WIDTH 128 // The width of the graph in pixels
#define GRAPH_HEIGHT 32 // The height of the graph in pixels
#define GRAPH_X 0 // The x coordinate of the graph origin
#define GRAPH_Y 32 // The y coordinate of the graph origin
#define GRAPH_MIN_AFR 10 // The minimum AFR value to display on the graph
#define GRAPH_MAX_AFR 20 // The maximum AFR value to display on the graph

// Define a circular buffer for storing the AFR values for the graph
float afr_buffer[GRAPH_WIDTH]; // The array that holds the AFR values
int afr_buffer_index = 0; // The index that points to the current position in the buffer

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Initialize SPI communication for OLED display:
  SPI.begin();

  // Initialize OLED display with SH1107 driver:
  oled.begin(SH1107_SWITCHCAPVCC, SH1107_I2C_ADDRESS);

  // Clear OLED display buffer:
  oled.clearDisplay();

  // Set text size and color for OLED display:
  oled.setTextSize(1);
  oled.setTextColor(WHITE);

  // Set injector pin as an input with a pull-up resistor:
  pinMode(INJ_PIN, INPUT_PULLUP);

}

void loop() {
  
   // Read analog value from oxygen sensor and convert it to voltage:
   o2_voltage = analogRead(O2_PIN) * (5.0 / 1023.0);

   // Compare oxygen sensor voltage with threshold and determine state:
   if (o2_voltage > O2_THRESHOLD) {
     o2_state = HIGH;
   } else {
     o2_state = LOW;
   }

   // Read digital value from injector signal and determine state:
   inj_state = digitalRead(INJ_PIN);

   // Check if there is a transition in oxygen sensor state:
   if (o2_state != o2_prev_state) {
     // If state changed from LOW to HIGH, record rise time:
     if (o2_state == HIGH) {
       o2_rise_time = micros();
     }
     // If state changed from HIGH to LOW, record fall time and calculate period and duty cycle:
     if (o2_state == LOW) {
       o2_fall_time = micros();
       float o2_period = (o2_fall_time - o2_rise_time) / 1000000.0; // Convert microseconds to seconds
       float o2_duty = (o2_fall_time - o2_rise_time) / (o2_fall_time - o2_rise_time + o2_rise_time - o2_fall_time);
       // Estimate air-fuel ratio using a linear approximation [^2^][5]:
       afr = map(o2_duty, 0.5, -0.5, 14.7, 22.4);
       // Update circular buffer with new AFR value:
       afr_buffer[afr_buffer_index] = afr;
       // Increment buffer index and wrap around if necessary:
       afr_buffer_index = (afr_buffer_index + 1) % GRAPH_WIDTH;
     }
     // Update previous state of oxygen sensor:
     o2_prev_state = o2_state;
   }

   // Check if there is a transition in injector signal state:
   if (inj_state != inj_prev_state) {
     // If state changed from LOW to HIGH, record rise time:
     if (inj_state == HIGH) {
       inj_rise_time = micros();
     }
     // If state changed from HIGH to LOW, record fall time and calculate period and duty cycle:
     if (inj_state == LOW) {
       inj_fall_time = micros();
       inj_period = (inj_fall_time - inj_rise_time) / 1000000.0; // Convert microseconds to seconds
       inj_duty = (inj_fall_time - inj_rise_time) / inj_period;
       // Estimate correction factor using a linear approximation [^2^][5]:
       corr = map(inj_duty, 0.5, -0.5, 1.0, 0.8);
     }
     // Update previous state of injector signal:
     inj_prev_state = inj_state;
   }

   // Print results on serial monitor:
   Serial.print("O2 voltage: ");
   Serial.print(o2_voltage);
   Serial.print(" V, AFR: ");
   Serial.print(afr);
   Serial.print(", Injector duty: ");
   Serial.print(inj_duty * 100);
   Serial.print(" %, Correction factor: ");
   Serial.println(corr);

   // Print results on OLED display:
   oled.setCursor(0, 0); // Set cursor to first column and first row
   oled.print("O2: ");
   oled.print(o2_voltage);
   oled.print("V ");
   oled.print(afr);
   oled.setCursor(0, 8); // Set cursor to first column and second row
   oled.print("INJ: ");
   oled.print(inj_duty * 100);
   oled.print("% ");
   oled.print(corr);

   // Draw graph on OLED display:
   
   // Clear the graph area of the display buffer:
   oled.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, BLACK);

   // Draw the horizontal axis of the graph:
   oled.drawLine(GRAPH_X, GRAPH_Y + GRAPH_HEIGHT / 2, GRAPH_X + GRAPH_WIDTH - 1, GRAPH_Y + GRAPH_HEIGHT / 2, WHITE);

   // Draw the vertical axis of the graph:
   oled.drawLine(GRAPH_X + GRAPH_WIDTH / 2, GRAPH_Y, GRAPH_X + GRAPH_WIDTH / 2, GRAPH_Y + GRAPH_HEIGHT - 1, WHITE);

   // Draw the graph line using the values from the circular buffer:
   
   // Declare variables for storing the previous and current pixel coordinates of the graph line:
   int prev_x = 0;
   int prev_y = 0;
   int curr_x = 0;
   int curr_y = 0;

   // Loop through the circular buffer starting from the current index:
   for (int i = 0; i < GRAPH_WIDTH; i++) {
     // Calculate the current pixel x coordinate based on the buffer index and the graph origin:
     curr_x = GRAPH_X + i;

     // Get the AFR value from the circular buffer at the current index:
     float curr_afr = afr_buffer[(afr_buffer_index + i) % GRAPH_WIDTH];

     // Map the AFR value to the pixel y coordinate based on the graph origin and scale:
     curr_y = map(curr_afr, GRAPH_MIN_AFR, GRAPH_MAX_AFR, GRAPH_Y + GRAPH_HEIGHT - 1, GRAPH_Y);

     // Draw a pixel at the current coordinates:
     oled.drawPixel(curr_x, curr_y, WHITE);

     // If this is not the first iteration, draw a line from the previous to the current coordinates:
     if (i > 0) {
       oled.drawLine(prev_x, prev_y, curr_x, curr_y, WHITE);
     }

     // Update the previous coordinates with the current ones:
     prev_x = curr_x;
     prev_y = curr_y;
   }

   // Display the OLED buffer on the screen:
   oled.display();

   // Wait for a short delay:
   //delay(100);
}
