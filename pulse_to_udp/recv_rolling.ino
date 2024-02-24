// Dreamed by copilot

// Define the OLED display parameters
#define OLED_WIDTH 128 // OLED display width, in pixels
#define OLED_HEIGHT 64 // OLED display height, in pixels
#define OLED_ADDRESS 0x3C // OLED display I2C address

// Include the necessary libraries
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// declare packet buffer size. should be power of 2 or it will not wrap
#define PACKET_BUFFER_SIZE 100 
// Declare a global variable to store the rolling buffer size
#define ROLLING_SIZE OLED_WIDTH

// Declare a pragma packed struct to store the pulse data
#pragma pack(push, 1)
struct PulseData {
  uint32_t rpm[PACKET_BUFFER_SIZE]; // Array to store the RPM values
  uint32_t length[PACKET_BUFFER_SIZE]; // Array to store the pulse length values
  uint8_t index; // Index to keep track of the array position
};
#pragma pack(pop)

// Declare a global variable to store the asyncUDP object
AsyncUDP udp;

// Declare a global variable to store the OLED display object
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// Declare a global variable to store the rolling buffer for pulse length
uint32_t rollingBuffer[ROLLING_SIZE];

// Declare a global variable to store the rolling buffer index
uint8_t rollingIndex = 0;

// Declare a function to handle the UDP packet
void handlePacket(AsyncUDPPacket packet);

// Declare a function to draw the line graph on the OLED display
void drawGraph();

// Setup function
void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);

  // Connect to the WiFi network
  WiFi.begin("your-ssid", "your-password");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize the asyncUDP object
  if (udp.listenMulticast(IPAddress(224, 0, 1, 187), UDP_PORT)) {
    Serial.println("UDP listening");
    // Set the callback function to handle the UDP packet
    udp.onPacket(handlePacket);
  }

  // Initialize the OLED display object
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Waiting for pulse data...");
  display.display();
}

// Loop function
void loop() {
  // Draw the line graph on the OLED display
  drawGraph();
}

// Function to handle the UDP packet
void handlePacket(AsyncUDPPacket packet) {
  // Declare a local variable to store the pulse data
  PulseData pulseData;

  // Copy the UDP packet data to the pulse data variable
  memcpy(&pulseData, packet.data(), sizeof(pulseData));

  // Print the pulse data to the serial monitor
  Serial.println("Pulse data received");
  Serial.print("index: ");
  Serial.println(pulseData.index);
  for (int i = 0; i < PACKET_BUFFER_SIZE; i++) {
    Serial.print("RPM: ");
    Serial.print(pulseData.rpm[i]);
    Serial.print(" Length: ");
    Serial.println(pulseData.length[i]);

    //Roll the rolling buffer.
    
    // Add the pulse length to the rolling buffer
    rollingBuffer[ROLLING_SIZE-1] = pulseData.length[i];

    // Increment the rolling buffer index and wrap around if needed
    //rollingIndex = (rollingIndex + 1) % ROLLING_SIZE;
  }
}

// Function to draw the line graph on the OLED display
void drawGraph() {
  // Clear the display
  display.clearDisplay();

  // Draw the graph points and lines
  for (int i = 0; i < ROLLING_SIZE; i++) {
    // Map the pulse length value to the display range
    int y = map(rollingBuffer[i], 0, 65000, OLED_HEIGHT, 0);

    // Draw a dot at the graph point
    display.drawPixel(i , y, WHITE);

    // Draw a line to the next graph point if it exists
    if (i < ROLLING_SIZE - 1) {
      int y2 = map(rollingBuffer[i + 1], 0, 65000, OLED_HEIGHT, 0);
      display.drawLine(i + 10, y, i + 11, y2, WHITE);
    }
  }

  // Update the display
  display.display();
}
