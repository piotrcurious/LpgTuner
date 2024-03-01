// Dreamed by copilot
#include "wifi_settings.h" // see to enable AP mode too
#include "display_settings.h" // include settings of the display 

uint32_t last_display_time = 0 ; // holder for last display refresh event
//#define DISPLAY_REFRESH_RATE 10 // display refresh rate in ms

#define GRAPH_LINE
//#define GRAPH_DOTS
//#define GRAPH_BARS
//#define GRAPH_TEST // test the display without incoming packets by initalizing with random data

// Include the necessary libraries
//#include <Arduino.h>
//#include <WiFi.h>
//#include <AsyncUDP.h>


// declare packet buffer size. 
#define PACKET_BUFFER_SIZE 2
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
//  Serial.begin(115200);

#ifdef MONO_OLED_128x64
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("injection pulse");
  display.display();
#endif //MONO_OLED_128x64

#ifdef st7735_tft
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(0, 0);
  tft.print("injection pulse");
//  tft.display();
#endif //st7735_tft

tft.setSPISpeed(70000000);

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
#ifdef MONO_OLED_128x64
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 8);
  display.print("WiFi: ");
  display.println(millis());
  display.display();
#endif // MONO_OLED_128x64
#ifdef st7735_tft
  tft.setTextSize(1);
  tft.setTextColor(ST7735_RED);
  tft.setCursor(0, 8);
  tft.print("WiFi: ");
  tft.print(millis());
#endif// st7735_tft
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
#ifdef MONO_OLED_128x64
  display.setTextSize(1);
  display.setTextColor(WHITE,BLACK);
  display.setCursor(0, 8);
  display.print("WiFi: ");
  display.print(millis());
  display.display();
#endif// MONO_OLED_128x64

#ifdef st7735_tft
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setCursor(0, 8);
  tft.print("WiFi: ");
  tft.setTextColor(ST7735_RED,ST7735_BLACK);
  tft.print(millis());
#endif// st7735_tft
  delay(50);

//    Serial.println("Connecting to WiFi...");

    }
//  display.clearDisplay();
//  display.display();   
#endif AP_mode_on 

  // Initialize the asyncUDP object
  if (udp.listenMulticast(multicastIP, multicastPort)) {
//    Serial.println("UDP listening");
//  display.clearDisplay();
#ifdef MONO_OLED_128x64
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  display.print("UDP: ");
  display.println(millis());
  display.display();
#endif MONO_OLED_128x64
#ifdef st7735_tft
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setCursor(0, 16);
  tft.print("UDP: ");
  tft.setTextColor(ST7735_GREEN,ST7735_BLACK);
  tft.println(millis());
#endif// st7735_tft
  delay(500);
    // Set the callback function to handle the UDP packet
    udp.onPacket(handlePacket);
  }

  // Initialize the OLED display object
//  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
//    Serial.println(F("SSD1306 allocation failed"));
//    for (;;);
//  }

#ifdef MONO_OLED_128x64
//  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 16+8);
  display.print("ready: ");
  display.println(millis());
  display.display();
#endif// MONO_OLED_128x64

#ifdef st7735_tft
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setCursor(0, 16+8);
  tft.print("ready: ");
  tft.setTextColor(ST7735_BLUE,ST7735_BLACK);
  tft.println(millis());
#endif// st7735_tft
  delay(1000);
}

// Loop function
void loop() {

#ifdef DISPLAY_REFRESH_RATE
  // Draw the line graph on the OLED display
  if (millis() - last_display_time >= DISPLAY_REFRESH_RATE ) { 
  drawGraph();
  last_display_time = millis();
  }
#endif // DISPLAY_REFRESH_RATE

#ifndef DISPLAY_REFRESH_RATE
  if (new_packet) {
  drawGraph();
  new_packet=false;
  }
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
  // Clear the display
  
#ifdef MONO_OLED_128x64  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
#endif // MONO_OLED_128x64
#ifdef st7735_tft
//  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
#endif// st7735_tft

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
#ifdef MONO_OLED_128x64
//    display.drawPixel(i , y, WHITE);
#endif // MONO_OLED_128x64
#ifdef st7735_tft
//    tft.drawPixel(i , y, ST7735_WHITE);
      tft.drawFastVLine(i+1,0,OLED_HEIGHT,ST7735_BLACK);
#endif// st7735_tft

    // Draw a line to the next graph point if it exists
    if (i < ROLLING_SIZE - 1) {
      int y2 = map(rollingBuffer[i + 1], graphMin, graphMax, OLED_HEIGHT-1, 0);
#ifdef MONO_OLED_128x64
      display.drawLine(i , y, i +1, y2, WHITE);
#endif// MONO_OLED_128x64
#ifdef st7735_tft
      tft.drawLine(i , y, i +1, y2, ST7735_WHITE);
#endif// st7735_tft

    }
  }

#ifdef MONO_OLED_128x64
  display.fillRect(0,0,5*8,8,BLACK);
  display.setCursor(0, 0);
  display.print(graphMax);
  display.fillRect(0,OLED_HEIGHT-8,5*8,8,BLACK);
  display.setCursor(0, OLED_HEIGHT-8);
  display.print(graphMin);

  display.drawRect(6*8,OLED_HEIGHT-8,4*8,8,BLACK);
  display.setCursor(6*8+1, OLED_HEIGHT-8);
  display.setTextColor(BLACK);
  display.print(pulseData.rpm[PACKET_BUFFER_SIZE-1]);
  display.setCursor(6*8, OLED_HEIGHT-8);
  display.setTextColor(2);
  display.print(pulseData.rpm[PACKET_BUFFER_SIZE-1]);

//  display.print(pulseData.rpm[1]);
//  display.setCursor(6*8, 0);
//  display.print(pulseData.index);

  display.setCursor(6*8, 0);
  display.print(pulseData.length[PACKET_BUFFER_SIZE-1]);
#endif // MONO_OLED_128x64



#endif //GRAPH_LINE

#ifdef GRAPH_DOTS
  // Draw the graph points and lines
  for (int i = 0; i < (ROLLING_SIZE-1); i++) {
    // Map the pulse length value to the display range
    int y = map(rollingBuffer[i], graphMin, graphMax, OLED_HEIGHT-1, 0);

    // Draw a dot at the graph point
#ifdef MONO_OLED_128x64
    display.drawPixel(i , y, WHITE);
#endif// MONO_OLED_128x64
#ifdef st7735_tft
    tft.drawPixel(i , y, ST7735_WHITE);
#endif// st7735_tft


    // Draw a line to the next graph point if it exists
//    if (i < ROLLING_SIZE - 1) {
//      int y2 = map(rollingBuffer[i + 1], graphMin, graphMax, OLED_HEIGHT-1, 0);
//      display.drawLine(i , y, i +1, y2, WHITE);
//    }

  }
#endif //GRAPH_DOTS

#ifdef GRAPH_BARS
  // Draw the graph data as vertical bars
  for (uint8_t i = 0; i < (ROLLING_SIZE-1); i++) {
    // Map the data value to the graph height
    uint8_t barHeight = map(rollingBuffer[i], graphMin, graphMax, 0, OLED_HEIGHT-1 );
    
    // Draw a vertical bar from the bottom to the data value
//    display.drawLine(graphX + i, graphY + graphH , graphX + i, graphY + graphH - barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY + graphH , barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY+(graphH-barHeight) , barHeight, SSD1306_WHITE);
#ifdef MONO_OLED_128x64
    display.drawFastVLine( i, (OLED_HEIGHT-barHeight) , barHeight, SSD1306_WHITE);   
#endif// MONO_OLED_128x64

  }

#ifdef MONO_OLED_128x64
//  display.fillRect(0,0,5*8,8,BLACK);
  display.setCursor(0, 0);
  display.setTextColor(2);
  display.print(graphMax);
  display.fillRect(0,OLED_HEIGHT-8,5*8,8,BLACK);
  display.setCursor(0, OLED_HEIGHT-8);
  display.print(graphMin);
  display.setCursor(6*8, 0);
  display.print(pulseData.length[PACKET_BUFFER_SIZE-1]);

#endif// MONO_OLED_128x64

#endif //GRAPH_BARS

  // Update the display
#ifdef MONO_OLED_128x64
  display.display();
#endif// MONO_OLED_128x64

}
