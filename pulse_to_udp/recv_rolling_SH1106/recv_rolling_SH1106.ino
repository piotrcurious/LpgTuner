// Dreamed by copilot
#include "wifi_settings.h" // see to enable AP mode too

// Define the OLED display parameters
#define OLED_WIDTH 128 // OLED display width, in pixels
#define OLED_HEIGHT 64 // OLED display height, in pixels
//#define OLED_ADDRESS 0x3C // OLED display I2C address
//#define OLED_BITRATE 8000000 // OLED SPI bitrate 8 Mhz default
#define OLED_BITRATE  80000000 // OLED SPI bitrate 80Mhz max
#define I2C_CLK_SPEED 1000000 // 1Mhz is max esp32 can do 
#define I2C_CLK_SPEED_POST 1000000 // setting post use of sh1106G display, for other peripherials sharing i2c bus.

#define TRANSLATION_TIMEBASE 24000 // timebase for translations from pulse count to ms
                                   // translation to ms is done in float realm. 
                                   

uint32_t last_display_time = 0 ; // holder for last display refresh event
//#define DISPLAY_REFRESH_RATE 30 // display refresh rate in ms 
                                //can be commented out entirely - the display rate is limited and synchronized to influx of udp packets then. 
                                // commenting out gives most fluidity on OLED displays where tearing is not an issue. 
                                

#define GRAPH_LINE        // line graph - fairly slow
//#define GRAPH_BARS          // faster
//#define GRAPH_DOTS        // fastest
//#define GRAPH_TEST // test the display without incoming packets by initalizing with random data
#define GRAPH_TIMING // draw time needed to display redraw

// Include the necessary libraries
//#include <Arduino.h>
//#include <WiFi.h>
//#include <AsyncUDP.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <Adafruit_SH110X.h>


// declare packet buffer size. 
#define PACKET_BUFFER_SIZE 2
// Declare a global variable to store the rolling buffer size
#define ROLLING_SIZE OLED_WIDTH

// Declare a pragma packed struct to store the pulse data
#pragma pack(push, 1)
struct PulseData {
//  uint32_t rpm[PACKET_BUFFER_SIZE]; // Array to store the RPM values
  float rpm[PACKET_BUFFER_SIZE]; // Array to store the RPM values
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

//#define OLED_DC     16
//#define OLED_CS     5
//#define OLED_RESET  17

//Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT,
//  &SPI, OLED_DC, OLED_RESET, OLED_CS, OLED_BITRATE);

#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define DISPLAY_WHITE SH110X_WHITE
#define DISPLAY_BLACK SH110X_BLACK

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
#define I2C_SCL 7 // 
#define I2C_SDA 8 // 
//Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET,100000,100000);
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET,I2C_CLK_SPEED,I2C_CLK_SPEED_POST);


  
//*/


// Declare a global variable to store the rolling buffer for pulse length
uint32_t rollingBuffer[ROLLING_SIZE];
uint32_t graphMin = 0;
uint32_t graphMax = 0;
// Declare a global variable to store the rolling buffer index
uint8_t rollingIndex = 0;

#ifdef GRAPH_TIMING
uint32_t graph_redraw_time = 0; 
uint32_t graph_redraw_time_last = 0; 
#endif// GRAPH_TIMING

bool new_packet = 0 ; // new packet flag

// Declare a function to handle the UDP packet
void handlePacket(AsyncUDPPacket packet);

// Declare a function to draw the line graph on the OLED display
void drawGraph();

// Setup function
void setup() {
  // Initialize the serial monitor
//  Serial.begin(115200);
//  Wire.setClock(400000L);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(2000000L);

//  display.begin(SSD1306_SWITCHCAPVCC);
  display.begin(i2c_Address, true); // Address 0x3C default
  Wire.setClock(2000000L);

 //display.setContrast (0); // dim display
 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(DISPLAY_WHITE);
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
  display.setTextSize(1);
  display.setTextColor(DISPLAY_WHITE);
  display.setCursor(0, 8);
  display.print("WiFi: ");
  display.println(millis());
  display.display();
  delay(500);

//  display.clearDisplay();

#endif AP_mode_on

#ifndef AP_mode_on // if not AP mode, start STA
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
//  display.setCursor(0, 0);     // Start at top-left corner
//  display.print("WiFi: ");
//  display.print(millis());
//  display.display();
  display.setTextSize(1);
  display.setTextColor(DISPLAY_WHITE,DISPLAY_BLACK);
  display.setCursor(0, 8);
  display.print("WiFi: ");
  display.print(millis());
  display.display();
  delay(50);

//    Serial.println("Connecting to WiFi...");

    }
//  display.clearDisplay();
//  display.display();   
#endif AP_mode_on 
esp_wifi_set_ps(WIFI_PS_NONE);


  // Initialize the asyncUDP object
  if (udp.listenMulticast(multicastIP, multicastPort)) {
//    Serial.println("UDP listening");
//  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(DISPLAY_WHITE);
  display.setCursor(0, 16);
  display.print("UDP: ");
  display.println(millis());
  display.display();
  delay(500);
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
  display.setTextColor(DISPLAY_WHITE);
  display.setCursor(0, 16+8);
  display.print("ready: ");
  display.println(millis());
  display.display();
  delay(1000);
}

// Loop function
void loop() {

#ifdef DISPLAY_REFRESH_RATE
  // Draw the line graph on the OLED display
  if (millis() - last_display_time >= DISPLAY_REFRESH_RATE ) { 
//  drawGraph();
  new_packet = false; 
  while(!new_packet) {yield();}; // wait for new packet to avoid screen tearing
                                // in case drawGraph would draw during packet update 
  if (new_packet) {
  drawGraph();
  new_packet=false;
  }
  last_display_time = millis();
  }
#endif // DISPLAY_REFRESH_RATE

#ifndef DISPLAY_REFRESH_RATE
  if (new_packet) {
  drawGraph();
  new_packet=false;
  } else {yield();}
//  delay(1);
#endif //DISPLAY_REFRESH_RATE
  
//  delay(100); // fixme : millis interval  
}

  // Declare a global variable to store the pulse data
  PulseData pulseData;


// Function to handle the UDP packet
IRAM_ATTR void handlePacket(AsyncUDPPacket packet) {
  // Declare a local variable to store the pulse data
//  PulseData pulseData;

  // Copy the UDP packet data to the pulse data variable
  memcpy((byte*)&pulseData, packet.data(), sizeof(pulseData));

  // Print the pulse data to the serial monitor
//  Serial.println("Pulse data received");
//  Serial.print("index: ");
//  Serial.println(pulseData.index);
//  for (int i = 0; i < (PACKET_BUFFER_SIZE -1); i++) {
  for (int i = 0; i < PACKET_BUFFER_SIZE; i++) {

//    Serial.print("RPM: ");
//    Serial.print(pulseData.rpm[i]);
//    Serial.print(" Length: ");
//    Serial.println(pulseData.length[i]);

    //Roll the rolling buffer.
    // Shift the graph data to the left by one pixel and find the maximum value
  graphMin = graphMax; // set graphMin to last graphMax value
  graphMax = 0;
  for (int i = 0; i < (ROLLING_SIZE - 1); i++) {

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

#ifdef GRAPH_TIMING // time accountig for debug/testing
graph_redraw_time = micros(); 
#endif// GRAPH_TIMING

  // Clear the display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(DISPLAY_WHITE);

#ifdef GRAPH_TEST
  for (int i = 0; i < (ROLLING_SIZE-1); i++) {
    // initalize array with bogus values
    rollingBuffer[i]=random(100);
  }
  graphMin= 0;
  graphMax= 100; 
#endif // GRAPH_TEST

#ifdef GRAPH_LINE
  // Draw the graph points and lines
  for (int i = 0; i < (ROLLING_SIZE-1); i++) {
    // Map the pulse length value to the display range
    int y = map(rollingBuffer[i], graphMin, graphMax, OLED_HEIGHT-1, 0);

    // Draw a dot at the graph point
    display.drawPixel(i , y, DISPLAY_WHITE);
    // Draw a line to the next graph point if it exists
    if (i < ROLLING_SIZE - 1) {
      int y2 = map(rollingBuffer[i + 1], graphMin, graphMax, OLED_HEIGHT-1, 0);
      display.drawLine(i , y, i +1, y2, DISPLAY_WHITE);
    }
  }

  display.fillRect(0,0,5*8,8,DISPLAY_BLACK);
  display.setCursor(0, 0);
  display.print((float)graphMax/TRANSLATION_TIMEBASE,4);
  display.fillRect(0,OLED_HEIGHT-8,5*8,8,DISPLAY_BLACK);
  display.setCursor(0, OLED_HEIGHT-8);
  display.print((float)graphMin/TRANSLATION_TIMEBASE,4);

  display.fillRect(6*8,OLED_HEIGHT-8,4*8,8,DISPLAY_BLACK);
//  display.setCursor(6*8+1, OLED_HEIGHT-8);
//  display.setTextColor(DISPLAY_BLACK);
//  display.print(pulseData.rpm[PACKET_BUFFER_SIZE-1]);
  display.setCursor(6*8, OLED_HEIGHT-8);
  display.setTextColor(2);
  display.print(pulseData.rpm[PACKET_BUFFER_SIZE-1],3);

//  display.print(pulseData.rpm[1]);
//  display.setCursor(6*8, 0);
//  display.print(pulseData.index);

  display.setCursor(6*8, 0);
  display.print((float)pulseData.length[PACKET_BUFFER_SIZE-1]/TRANSLATION_TIMEBASE,4);

#endif //GRAPH_LINE

#ifdef GRAPH_DOTS
  // Draw the graph points and lines
  for (int i = 0; i < (ROLLING_SIZE-1); i++) {
    // Map the pulse length value to the display range
    int y = map(rollingBuffer[i], graphMin, graphMax, OLED_HEIGHT-1, 0);

    // Draw a dot at the graph point
    display.drawPixel(i , y, DISPLAY_WHITE);
    // Draw a line to the next graph point if it exists
//    if (i < ROLLING_SIZE - 1) {
//      int y2 = map(rollingBuffer[i + 1], graphMin, graphMax, OLED_HEIGHT-1, 0);
//      display.drawLine(i , y, i +1, y2, DISPLAY_WHITE);
//    }

  }
#endif //GRAPH_DOTS

#ifdef GRAPH_BARS
  // Draw the graph data as vertical bars
  for (uint8_t i = 0; i < (ROLLING_SIZE-1); i++) {
    // Map the data value to the graph height
    uint8_t barHeight = map(rollingBuffer[i], graphMin, graphMax, 0, OLED_HEIGHT-1 );
    
    // Draw a vertical bar from the bottom to the data value
//    display.drawLine(graphX + i, graphY + graphH , graphX + i, graphY + graphH - barHeight, DISPLAY_WHITE);
    //display.drawFastVLine(graphX + i, graphY + graphH , barHeight, DISPLAY_WHITE);
    //display.drawFastVLine(graphX + i, graphY+(graphH-barHeight) , barHeight, DISPLAY_WHITE);
    display.drawFastVLine( i, (OLED_HEIGHT-barHeight) , barHeight, DISPLAY_WHITE);   
  }
//  display.fillRect(0,0,5*8,8,DISPLAY_BLACK);
  display.setCursor(0, 0);
  display.setTextColor(2);
  display.print(graphMax);
  display.fillRect(0,OLED_HEIGHT-8,5*8,8,DISPLAY_BLACK);
  display.setCursor(0, OLED_HEIGHT-8);
  display.print(graphMin);
  display.setCursor(6*8, 0);
  display.print(pulseData.length[PACKET_BUFFER_SIZE-1]);

#endif //GRAPH_BARS
  // Update the display
//  display.display();
  
#ifdef GRAPH_TIMING // time accountig for debug/testing
//graph_redraw_time = millis(); 
  display.fillRect(OLED_WIDTH-4*8,0,4*8,8,DISPLAY_WHITE);
  display.setCursor(OLED_WIDTH-4*8,0);
  display.setTextColor(DISPLAY_BLACK);
//  display.print(micros()-graph_redraw_time);
  display.print(graph_redraw_time_last); // use last frame redraw time
  display.display();
  graph_redraw_time_last = micros()-graph_redraw_time;

#else 
  display.display();  // just display stuff
#endif// GRAPH_TIMING



}
