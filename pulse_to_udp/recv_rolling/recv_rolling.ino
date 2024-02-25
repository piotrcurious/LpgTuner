// Dreamed by copilot
#include "wifi_settings.h" // see to enable AP mode too

// Define the OLED display parameters
#define OLED_WIDTH 128 // OLED display width, in pixels
#define OLED_HEIGHT 64 // OLED display height, in pixels
//#define OLED_ADDRESS 0x3C // OLED display I2C address

#define GRAPH_LINE
//#define GRAPH_BARS
//#define GRAPH_TEST // test the display without incoming packets by initalizing with random data


// Include the necessary libraries
//#include <Arduino.h>
//#include <WiFi.h>
//#include <AsyncUDP.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// declare packet buffer size. 
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
//Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

/*
//#define OLED_DC     D2
#define OLED_DC     4  // D2 = gpio4 
//#define OLED_CS     D8
#define OLED_CS     15 // D8 = gpio15
//#define OLED_RESET  D3
#define OLED_RESET  0 //  D3 = gpio0
*/

#define OLED_DC     16
#define OLED_CS     5
#define OLED_RESET  17

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT,
  &SPI, OLED_DC, OLED_RESET, OLED_CS);
//*/


// Declare a global variable to store the rolling buffer for pulse length
uint32_t rollingBuffer[ROLLING_SIZE];
uint32_t graphMin = 0;
uint32_t graphMax = 0;
// Declare a global variable to store the rolling buffer index
uint8_t rollingIndex = 0;

bool new_packet = 0 ; // new packet flag

// Declare a function to handle the UDP packet
void handlePacket(AsyncUDPPacket packet);

// Declare a function to draw the line graph on the OLED display
void drawGraph();

// Setup function
void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("injection pulse");
  display.display();
  delay(1000);

  WiFi.persistent(false); // no need to wear off the flash as we have all data in the sketch

/*
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
*/

#ifdef AP_mode_on // if ap mode , start and configure AP
  WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
//  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 8);
  display.println("AP MODE");
  display.display();
  delay(1000);

#endif AP_mode_on

#ifndef AP_mode_on // if not AP mode, start STA
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
//  display.setCursor(0, 0);     // Start at top-left corner
//  display.print("WiFi: ");
//  display.print(millis());
//  display.display();
    delay(1000);
//    Serial.println("Connecting to WiFi...");

    }
//  display.clearDisplay();
//  display.display();   
#endif AP_mode_on 


  // Initialize the asyncUDP object
  if (udp.listenMulticast(multicastIP, multicastPort)) {
    Serial.println("UDP listening");
    // Set the callback function to handle the UDP packet
    udp.onPacket(handlePacket);
  }

  // Initialize the OLED display object
//  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
//    Serial.println(F("SSD1306 allocation failed"));
//    for (;;);
//  }

//  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  display.println("ready");
  display.display();
  delay(1000);
}

// Loop function
void loop() {
  // Draw the line graph on the OLED display
  drawGraph();
  delay(100); // fixme : millis interval  
}

// Function to handle the UDP packet
IRAM_ATTR void handlePacket(AsyncUDPPacket packet) {
  // Declare a local variable to store the pulse data
  PulseData pulseData;

  // Copy the UDP packet data to the pulse data variable
  memcpy((byte*)&pulseData, packet.data(), sizeof(pulseData));

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
    // Shift the graph data to the left by one pixel and find the maximum value
  graphMin = graphMax; // set graphMin to last graphMax value
  graphMax = 0;
  for (int i = 0; i < ROLLING_SIZE - 1; i++) {
    rollingBuffer[i] = rollingBuffer[i + 1];
    if (rollingBuffer[i] > graphMax) {
      graphMax = rollingBuffer[i];
      }
    if (rollingBuffer[i] < graphMin) {
      graphMin = rollingBuffer[i];
    }
    // graphMax = max(graphMax, graphData[i]); 
    // or use that instead 
  }
    // Add the pulse length to the rolling buffer
    rollingBuffer[ROLLING_SIZE-1] = pulseData.length[i];

    // Increment the rolling buffer index and wrap around if needed
    //rollingIndex = (rollingIndex + 1) % ROLLING_SIZE;
  }
  new_packet = true; 
}

// Function to draw the line graph on the OLED display
void drawGraph() {
  // Clear the display
  display.clearDisplay();

#ifdef GRAPH_TEST

  for (int i = 0; i < ROLLING_SIZE; i++) {
    // initalize array with bogus values
    rollingBuffer[i]=random(100);
  }
  graphMin= 0;
  graphMax= 100; 
#endif // GRAPH_TEST


#ifdef GRAPH_LINE
  // Draw the graph points and lines
  for (int i = 0; i < ROLLING_SIZE; i++) {
    // Map the pulse length value to the display range
    int y = map(rollingBuffer[i], graphMin, graphMax, OLED_HEIGHT, 0);

    // Draw a dot at the graph point
    display.drawPixel(i , y, WHITE);

    // Draw a line to the next graph point if it exists
    if (i < ROLLING_SIZE - 1) {
      int y2 = map(rollingBuffer[i + 1], graphMin, graphMax, OLED_HEIGHT, 0);
      display.drawLine(i , y, i +1, y2, WHITE);
    }
  }

#endif //GRAPH_LINE

#ifdef GRAPH_BARS
  // Draw the graph data as vertical bars
  for (uint8_t i = 0; i < ROLLING_SIZE; i++) {
    // Map the data value to the graph height
    uint8_t barHeight = map(rollingBuffer[i], graphMin, graphMax, 0, OLED_HEIGHT );
    
    // Draw a vertical bar from the bottom to the data value
//    display.drawLine(graphX + i, graphY + graphH , graphX + i, graphY + graphH - barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY + graphH , barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY+(graphH-barHeight) , barHeight, SSD1306_WHITE);
    display.drawFastVLine( i, (OLED_HEIGHT-barHeight) , barHeight, SSD1306_WHITE);   
  }

#endif //GRAPH_BARS

  // Update the display
  display.display();
}
