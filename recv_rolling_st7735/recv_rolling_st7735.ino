
// Dreamed by copilot
#include "wifi_settings.h" // see to enable AP mode too
#include "display_settings.h" // include settings of the display 


#define TRANSLATION_TIMEBASE 24000 // timebase for translations from pulse count to ms
                                   // translation to ms is done in float realm. 

#define GRAPH_LINE
//#define GRAPH_DOTS
//#define GRAPH_BARS
//#define GRAPH_TEST // test the display without incoming packets by initalizing with random data
#define GRAPH_TIMING // draw time needed to display redraw

// Include the necessary libraries
//#include <Arduino.h>
//#include <WiFi.h>
//#include <AsyncUDP.h>

// declare packet buffer size. 
#define PACKET_BUFFER_SIZE 2
// Declare a global variable to store the rolling buffer size
#ifdef st7735_tft
#define ROLLING_SIZE_TFT TFT_WIDTH
#endif// st7735_tft

#ifdef MONO_OLED_128x64
#define ROLLING_SIZE_OLED OLED_WIDTH
#endif// MONO_OLED_128x64

// Declare a pragma packed struct to store the pulse data
#pragma pack(push, 1)
struct PulseData {
  float rpm[PACKET_BUFFER_SIZE]; // Array to store the RPM values
  uint32_t length[PACKET_BUFFER_SIZE]; // Array to store the pulse length values
  uint8_t index; // Index to keep track of the array position
};
#pragma pack(pop)

// Declare a global variable to store the asyncUDP object
AsyncUDP udp;

// Declare a global variable to store the rolling buffer for pulse length
#ifdef MONO_OLED_128x64
uint32_t rollingBuffer_oled[ROLLING_SIZE_OLED];
uint32_t graphMin_oled = 0;
uint32_t graphMax_oled = 0;
// Declare a global variable to store the rolling buffer index
//uint8_t rollingIndex_oled = 0;

#endif// MONO_OLED_128x64

#ifdef st7735_tft
uint32_t rollingBuffer_tft[ROLLING_SIZE_TFT];
uint32_t graphMin_tft = 0;
uint32_t graphMax_tft = 0;
// Declare a global variable to store the rolling buffer index
//uint8_t rollingIndex_tft = 0;
#endif// st7735_tft

#ifdef GRAPH_TIMING
uint32_t graph_redraw_time = 0; 
#endif// GRAPH_TIMING

bool new_packet = 0 ; // new packet flag

// Declare a function to handle the UDP packet
void handlePacket(AsyncUDPPacket packet);

// Declare a function to draw the line graph on the OLED display
void drawGraph();

// Setup function
void setup() {
  // Initialize the serial monitor
  Serial.begin(115200); // fixme - actually not needed if no debug function needs it, but needs logic to be enabled if. 

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
//  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
//  tft.initR(INITR_GREENTAB);   // initialize a ST7735S chip, black tab

  tft.init();
  tft.initDMA();
  pinMode(TFT_LED,OUTPUT); // driving LED of tft
  digitalWrite(TFT_LED,LOW); // active low . it drives pnp transistor base via 1k resistor 
  tft.writecommand(ST7735_FRMCTR1); // frame rate control
  tft.writedata(0x0f);          // fastest refresh
  tft.writedata(0x2f);          // 6 lines front porch
  tft.writedata(0x2f);          // 3 lines backporch
//  tft.writecommand(ST7735_INVOFF); // voodoo from datasheet. it should be needed if moving back from FRMCTL2 to FRMCTL1  
// but the datasheet is not really very accurate on when those registers are needed, how they are updated, 
// when display actually reacts to changes (after full frame refresh is finished? or maybe earlier?) etc. 

  // set display to slowest refresh

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
//  tft.setTextColor(ST7735_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.print("injection pulse");
  tft.endWrite();
  
  // Define sprite colour depth
  spr[0].setColorDepth(COLOR_DEPTH);
  spr[1].setColorDepth(COLOR_DEPTH);

  // Create the 2 sprites for full screen framebuffer
  sprPtr[0] = (uint16_t*)spr[0].createSprite(IWIDTH, IHEIGHT);
  sprPtr[1] = (uint16_t*)spr[1].createSprite(IWIDTH, IHEIGHT);

//  tft.display();
#endif //st7735_tft

//tft.setSPISpeed(TFT_BITRATE); // for adafruit libraries

  delay(200);
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
  tft.startWrite();
  tft.setTextSize(1);
  tft.setTextColor(ST7735_RED);
  tft.setCursor(0, 8);
  tft.print("WiFi: ");
  tft.print(millis());
  tft.endWrite();

#endif// st7735_tft
  delay(200);

//  display.clearDisplay();

#endif AP_mode_on

#ifndef AP_mode_on // if not AP mode, start STA
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
#ifdef MONO_OLED_128x64
  display.setTextSize(1);
  display.setTextColor(WHITE,BLACK);
  display.setCursor(0, 8);
  display.print("WiFi: ");
  display.print(millis());
  display.display();
#endif// MONO_OLED_128x64

#ifdef st7735_tft
  tft.startWrite();
  tft.setTextSize(1);
//  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(0, 8);
  tft.print("WiFi: ");
//  tft.setTextColor(ST7735_RED,ST7735_BLACK);
  tft.setTextColor(TFT_RED,TFT_BLACK);
  tft.print(millis());
  tft.endWrite();

#endif// st7735_tft
  delay(50);

//    Serial.println("Connecting to WiFi...");

    }
//  display.clearDisplay();
//  display.display();   
#endif AP_mode_on 

esp_wifi_set_ps(WIFI_PS_NONE);
//esp_wifi_set_ps(WIFI_PS_MIN_MODEM); //just one in the network causes all others to chop!
//WiFi.setSleep(false); // same as above, no deps on external libs
WiFi.setSleep(WIFI_PS_NONE); // same as above, no deps on external libs

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
  tft.startWrite();
  tft.setTextSize(1);
//  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);

  tft.setCursor(0, 16);
  tft.print("UDP: ");
//  tft.setTextColor(ST7735_GREEN,ST7735_BLACK);
  tft.setTextColor(TFT_GREEN,TFT_BLACK);
  tft.println(millis());
  tft.endWrite();

#endif// st7735_tft
  delay(200);
    // Set the callback function to handle the UDP packet
    udp.onPacket(handlePacket);
  }

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
  tft.startWrite();
  tft.setTextSize(1);
//  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(0, 16+8);
  tft.print("ready: ");
//  tft.setTextColor(ST7735_BLUE,ST7735_BLACK);
  tft.setTextColor(TFT_BLUE,TFT_BLACK);
  tft.println(millis());
  tft.endWrite();

#endif// st7735_tft
  delay(200);
}

#ifdef TEARING_DEBUG
void serial_tft_helper () {
    // Check if there is any serial input
  if (Serial.available() > 0) {
    // Read the incoming byte
    char input = Serial.read();

    // Check if the input is a digit from 1 to 6
    if (input >= '1' && input <= '6') {
      // Set the selected variable to the corresponding one
      selected = input - '0';
    }

    // Check if the input is a minus sign
    else if (input == '-') {
      // Decrement the selected variable by 1
      switch (selected) {
        case 1:
          var1--;
          break;
        case 2:
          var2--;
          break;
        case 3:
          var3--;
          break;
        case 4:
          var4--;
          break;
        case 5:
          var5--;
          break;
        case 6:
          var6--;
          break;
      }
    }

    // Check if the input is an equal sign
    else if (input == '=') {
      // Increment the selected variable by 1
      switch (selected) {
        case 1:
          var1++;
          break;
        case 2:
          var2++;
          break;
        case 3:
          var3++;
          break;
        case 4:
          var4++;
          break;
        case 5:
          var5++;
          break;
        case 6:
          var6++;
          break;
      }
    }

    // Print all the variables to the serial monitor
    Serial.print("1 RTNA1 = ");
    Serial.print(var1,HEX);
    Serial.print(", 2 Fporch1= ");
    Serial.print(var2,HEX);
    Serial.print(", 3 Bporch1 = ");
    Serial.print(var3,HEX);
    Serial.print(", 4 RTNA2 = ");
    Serial.print(var4,HEX);
    Serial.print(", 5 Fporch2 = ");
    Serial.print(var5,HEX);
    Serial.print(", 6 Bporch2 = ");
    Serial.print(var6,HEX);

    // Print the selected variable with an asterisk
    Serial.print(", selected = ");
    Serial.print(selected);
    Serial.println("*");
  }
}

#endif//#ifdef TEARING_DEBUG

// Loop function
void loop() {
#ifdef TEARING_DEBUG
  serial_tft_helper();  // use to solve st7735 tearing problem
#endif// #ifdef TEARING_DEBUG

#ifdef DISPLAY_REFRESH_RATE
  // Draw the line graph on the OLED display
  if (millis() - last_display_time >= DISPLAY_REFRESH_RATE ) { 
//  drawGraph();
//  new_packet = false; 
  last_display_time = millis();

  tft.startWrite();

  tft.writecommand(ST7735_FRMCTR1); // frame rate control
#ifdef TEARING_DEBUG
  tft.writedata(var1);          // fastest refresh
  tft.writedata(var2);          // 6 lines front porch
  tft.writedata(var3);          // 3 lines backporch
#endif //#ifdef TEARING_DEBUG

#ifndef TEARING_DEBUG
  tft.writedata(0x0b);          // fastest refresh
  tft.writedata(0x16);          // 6 lines front porch
  tft.writedata(0x3f);          // 3 lines backporch
#endif //#ifndef TEARING_DEBUG
  odd_frame_adjusted = false;  // reset the odd frame adjustment flag
  odd_frame = !odd_frame ; // flip the odd_frame flag


//  while(!new_packet) {yield();}; // wait for new packet to avoid screen tearing
                                // in case drawGraph would draw during packet update 

    if (new_packet) {
//      if (true){ //each time . 
//    digitalWrite(TFT_LED,HIGH); // disable screen during DMA transfer
//    delay(10);
//    tft.writecommand( ST7735_DISPOFF ) ;
//    tft.writecommand( ST7735_SLPIN ) ;
//    tft.writecommand( ST7735_DISPON ) ;
  //  delay(20);
//  tft.writecommand(ST7735_FRMCTR1); // frame rate control
#ifdef TEARING_DEBUG
//  tft.writedata(var1);          // fastest refresh
//  tft.writedata(var2);          // 6 lines front porch
//  tft.writedata(var3);          // 3 lines backporch
#endif //#ifdef TEARING_DEBUG

#ifndef TEARING_DEBUG
//  tft.writedata(0x0b);          // fastest refresh
//  tft.writedata(0x16);          // 6 lines front porch
//  tft.writedata(0x3f);          // 3 lines backporch
#endif //#ifndef TEARING_DEBUG
//  odd_frame_adjusted = false;  // reset the odd frame adjustment flag
//  tft.writecommand(ST7735_INVOFF); 
// needed if switching from FRMCTL2 to FRMCTL1

//  tft.writecommand( ST7735_DISPOFF ) ;
//  tft.writecommand( ST7735_DISPON ) ; 
      drawGraph();
//  while (millis() - last_display_time <= DISPLAY_REFRESH_RATE+18 ) { 
//                delay(1);
//             }    
      new_packet=false;
    } //else {delay(2);};
    
  //last_display_time = millis();
  }
      if (tft.dmaBusy()) {  
//    digitalWrite(TFT_LED,HIGH); // disable screen during DMA transfer
  } else { 

//if (sprSel && !odd_frame_adjusted) {
if (millis() - last_display_time >= DISPLAY_REFRESH_RATE/2 ) { 
if (odd_frame && !odd_frame_adjusted) {

    tft.writecommand(ST7735_FRMCTR1); // frame rate control
  #ifdef TEARING_DEBUG
  tft.writedata(var4);          // fastest refresh
  tft.writedata(var5);          // 6 lines front porch
  tft.writedata(var6);          // 3 lines backporch
  #endif// #ifdef TEARING_DEBUG

  #ifndef TEARING_DEBUG
  tft.writedata(0x0a);          // fastest refresh
  tft.writedata(0x20);          // 6 lines front porch
  tft.writedata(0x3f);          // 3 lines backporch
  //  tft.writecommand(ST7735_INVOFF); // voodoo does not work... needed from FRMCTL2 to FRMCTL1
  #endif // #ifndef TEARING_DEBUG
  tft.endWrite(); // release exclusive use of SPI bus
  odd_frame_adjusted = true; 
}
}
//         tft.endWrite(); // release exclusive use of SPI bus
         }
  
#endif // DISPLAY_REFRESH_RATE

#ifndef DISPLAY_REFRESH_RATE
  if (new_packet) {
  if (!tft.dmaBusy()) {
//  digitalWrite(TFT_LED,HIGH); // disable screen during DMA transfer
//  tft.writecommand( ST7735_DISPOFF ) ;
//  tft.writecommand(ST7735_FRMCTR3);
//  tft.writecommand(ST7735_FRMCTR1); // frame rate control
//  tft.writedata(0x0F);          // fastest refresh
//  tft.writedata(0x1F);          // 6 lines front porch
//  tft.writedata(0x1F);          // 3 lines backporch
//  delay(1);
//  tft.writecommand(ST7735_FRMCTR1); // frame rate control
//  tft.writedata(0x01);          // fastest refresh
//  tft.writedata(0x6);          // 6 lines front porch
//  tft.writedata(0x3);          // 3 lines backporch
//  tft.writecommand(ST7735_INVOFF); 

//  delay(1);
//  delay(10);
  tft.writecommand(ST7735_FRMCTR1); // frame rate control
  tft.writedata(0x01);          // fastest refresh
  tft.writedata(0x1);          // 6 lines front porch
  tft.writedata(0x1);          // 3 lines backporch
  tft.writecommand(ST7735_INVOFF); 

//  tft.writecommand(ST7735_FRMCTR1); // frame rate control
//  tft.writedata(0x1F);          // fastest refresh
//  tft.writedata(0x3F);          // 6 lines front porch
//  tft.writedata(0x3F);          // 3 lines backporch
//  tft.writecommand(ST7735_INVOFF); 

//  delay(30);

//  delay(10);
//  tft.writecommand(ST7735_PTLAR); // frame rate control
//  tft.writedata(0x01);          // fastest refresh
//  tft.writedata(0x01);          // 6 lines front porch
//  tft.writedata(0x00);          // 3 lines backporch
//  tft.writedata(0x00);          // 3 lines backporch

//  tft.writecommand(ST7735_PTLON); // frame rate control

  drawGraph();

//  tft.writecommand(ST7735_FRMCTR1); // frame rate control
//  tft.writedata(0x0F);          // fastest refresh
//  tft.writedata(0x1F);          // 6 lines front porch
//  tft.writedata(0x1F);          // 3 lines backporch
//  delay(100);

//  tft.writecommand(ST7735_FRMCTR1); // frame rate control
//  tft.writedata(0x01);          // fastest refresh
//  tft.writedata(0x6);          // 6 lines front porch
//  tft.writedata(0x3);          // 3 lines backporch

//  tft.writecommand( ST7735_DISPON ) ;
  new_packet=false;
  }
  } else {yield();}

  if (tft.dmaBusy()) {
 //   digitalWrite(TFT_LED,HIGH); // disable screen during DMA transfer
  } else { 
 //        digitalWrite(TFT_LED,LOW); // enable screen when DMA is idle
         tft.endWrite(); // release exclusive use of SPI bus
  tft.writecommand(ST7735_FRMCTR1); // frame rate control
  tft.writedata(0x02);          // fastest refresh
  tft.writedata(0x2f);          // 6 lines front porch
  tft.writedata(0x2f);          // 3 lines backporch
  tft.writecommand(ST7735_INVOFF); 

//          delay(1);
//        digitalWrite(TFT_LED,HIGH); // disable screen during DMA transfer
//          delay(5);
         }

#endif //!DISPLAY_REFRESH_RATE
 

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

#ifdef MONO_OLED_128x64

//  for (int i = 0; i < (PACKET_BUFFER_SIZE -1); i++) {
  for (int i = 0; i < PACKET_BUFFER_SIZE; i++) {

//    Serial.print("RPM: ");
//    Serial.print(pulseData.rpm[i]);
//    Serial.print(" Length: ");
//    Serial.println(pulseData.length[i]);

    //Roll the rolling buffer.
    // Shift the graph data to the left by one pixel and find the maximum value
  graphMin_oled = graphMax_oled; // set graphMin to last graphMax value
  graphMax_oled = 0;
  for (int i = 0; i < (ROLLING_SIZE_OLED - 1); i++) {

    rollingBuffer_oled[i] = rollingBuffer_oled[i + 1];
    if (rollingBuffer_oled[i] > graphMax_oled) {
      graphMax_oled = rollingBuffer_oled[i];
      }
    if (rollingBuffer_oled[i] < graphMin_oled) {
      graphMin_oled = rollingBuffer_oled[i];
    }
    // graphMax_oled = max(graphMax_oled, rollingBuffer_oled[i]); 
    // or use that instead 
  }
    // Add the pulse length to the rolling buffer    
    rollingBuffer_oled[ROLLING_SIZE_OLED-1] = pulseData.length[i];

    // Increment the rolling buffer index and wrap around if needed
    //rollingIndex_oled = (rollingIndex_oled + 1) % ROLLING_SIZE;
  }
#endif// MONO_OLED_128x64

#ifdef st7735_tft
//  for (int i = 0; i < (PACKET_BUFFER_SIZE -1); i++) {
  for (int i = 0; i < PACKET_BUFFER_SIZE; i++) {

//    Serial.print("RPM: ");
//    Serial.print(pulseData.rpm[i]);
//    Serial.print(" Length: ");
//    Serial.println(pulseData.length[i]);

    //Roll the rolling buffer.
    // Shift the graph data to the left by one pixel and find the maximum value
  graphMin_tft = graphMax_tft; // set graphMin to last graphMax value
  graphMax_tft = 0;
  for (int i = 0; i < (ROLLING_SIZE_TFT - 1); i++) {

    rollingBuffer_tft[i] = rollingBuffer_tft[i + 1];
    if (rollingBuffer_tft[i] > graphMax_tft) {
      graphMax_tft = rollingBuffer_tft[i];
      }
    if (rollingBuffer_tft[i] < graphMin_tft) {
      graphMin_tft = rollingBuffer_tft[i];
    }
    // graphMax_tft = max(graphMax_tft, rollingBuffer_tft[i]); 
    // or use that instead 
  }
    // Add the pulse length to the rolling buffer    
    rollingBuffer_tft[ROLLING_SIZE_TFT-1] = pulseData.length[i];

    // Increment the rolling buffer index and wrap around if needed
    //rollingIndex_tft = (rollingIndex_tft + 1) % ROLLING_SIZE;
  }
#endif// st7735_tft
  new_packet = true; 
}

// Function to draw the line graph on the OLED display
IRAM_ATTR void drawGraph() {
#ifdef GRAPH_TIMING // time accountig for debug/testing
graph_redraw_time = micros(); 
#endif// GRAPH_TIMING
  
  // Clear the display  
#ifdef MONO_OLED_128x64  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
#endif // MONO_OLED_128x64
#ifdef st7735_tft
//tft.fillScreen(ST7735_BLACK);
//tft.fillScreen(TFT_BLACK);

//  tft.setTextSize(1);
//  tft.setTextColor(ST7735_WHITE);
//  tft.setTextColor(TFT_WHITE);

  // Grab exclusive use of the SPI bus
//  tft.startWrite();
//  tft.init();
//tft.writecommand( ST7735_SLPIN ) ;
//tft.writecommand( ST7735_DISPOFF ) ;

//delay(1);
//tft.writecommand( ST7735_SLPOUT ) ;
//delay(1);
  
#ifdef USE_DMA_TO_TFT
//      if (tft.dmaBusy()) prime_max++; // Increase processing load until just not busy
      tft.pushImageDMA(0, 0, IWIDTH, IHEIGHT, sprPtr[sprSel]);
      sprSel = !sprSel;
#else
      spr[sprSel].pushSprite(0, 0); // Blocking write (no DMA) 115fps
      sprSel = !sprSel;
#endif

//  if (tft.dmaBusy()) {
//    digitalWrite(TFT_LED,HIGH); // disable screen during DMA transfer
//  } else { 
//         digitalWrite(TFT_LED,LOW); // enable screen when DMA is idle
//         }

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

#ifdef MONO_OLED_128x64
  // Draw the graph points and lines
  for (int i = 0; i < (ROLLING_SIZE_OLED-1); i++) {
    // Map the pulse length value to the display range
    int y = map(rollingBuffer_oled[i], graphMin, graphMax, OLED_HEIGHT-1, 0);

    // Draw a dot at the graph point
//    display.drawPixel(i , y, WHITE);

    // Draw a line to the next graph point if it exists
    if (i < ROLLING_SIZE - 1) {
      uint8_t y2 = map(rollingBuffer_oled[i + 1], graphMin, graphMax, OLED_HEIGHT-1, 0);
      display.drawLine(i , y, i +1, y2, WHITE);
    }
  }
#endif// MONO_OLED_128x64

#ifdef st7735_tft
  // Draw the graph points and lines
//  tft.drawFastVLine(0,0,TFT_HEIGHT,ST7735_BLACK); // clear vertical line ahead to avoid flicker (centipede instead)
//  tft.drawFastVLine(0,0,TFT_HEIGHT,TFT_BLACK); // clear vertical line ahead to avoid flicker (centipede instead)

//   tft.fillScreen(TFT_BLACK); // clear screen (causes flicker)
   if (sprSel){
   spr[sprSel].fillScreen(TFT_BLACK); // clear screen on framebuffer 
   } else {
//   spr[sprSel].fillScreen(TFT_RED); // clear screen on framebuffer 
   spr[sprSel].fillScreen(TFT_BLACK); // clear screen on framebuffer 
   }

  for (uint8_t i = 0; i < (ROLLING_SIZE_TFT-1); i++) {
    // Map the pulse length value to the display range
    uint8_t y = map(rollingBuffer_tft[i], graphMin_tft, graphMax_tft, TFT_HEIGHT-1, 0);

//      tft.drawPixel(i , y, ST7735_WHITE);
//      tft.drawFastVLine(i+1,0,TFT_HEIGHT,ST7735_BLACK); // clear vertical line ahead to avoid flicker (centipede instead)
//      tft.drawFastVLine(i+1,0,TFT_HEIGHT,TFT_BLACK); // clear vertical line ahead to avoid flicker (centipede instead)

    // Draw a line to the next graph point if it exists
    if (i < ROLLING_SIZE_TFT - 1) {
      uint8_t y2 = map(rollingBuffer_tft[i + 1], graphMin_tft, graphMax_tft, TFT_HEIGHT-1, 0);
//      tft.drawLine(i , y, i +1, y2, ST7735_WHITE);
//      tft.drawLine(i , y, i +1, y2, TFT_WHITE);

        spr[sprSel].drawLine(i , y, i +1, y2, TFT_WHITE);
//      spr[sprSel].drawWedgeLine(i, y, i+1, y2, 0, 0, TFT_WHITE, TFT_MAROON);
//        spr[sprSel].drawWedgeLine(i, y, i+1, y2, 0.1, 0.1, TFT_WHITE);
//      spr[sprSel].drawWedgeLine(i, y, i+1, y2, 0.1, 0.1, TFT_MAROON,TFT_WHITE);
//      spr[sprSel].drawWedgeLine(i, y, i+1, y2, 0.01, 0.01, TFT_WHITE,TFT_BLUE);
//      spr[sprSel].drawWedgeLine(i, y, i+1, y2, 0.3, 0.3, TFT_WHITE);
//      spr[sprSel].drawWideLine(i, y, i+1, y2, 0.5, TFT_WHITE);


    }
  }
#endif// st7735_tft

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

#ifdef MONO_OLED_128x64
  // Draw the graph data as vertical bars
  for (uint8_t i = 0; i < (ROLLING_SIZE-1); i++) {
    // Map the data value to the graph height
    uint8_t barHeight = map(rollingBuffer[i], graphMin, graphMax, 0, OLED_HEIGHT-1 );
    
    // Draw a vertical bar from the bottom to the data value
//    display.drawLine(graphX + i, graphY + graphH , graphX + i, graphY + graphH - barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY + graphH , barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY+(graphH-barHeight) , barHeight, SSD1306_WHITE);
    display.drawFastVLine( i, (OLED_HEIGHT-barHeight) , barHeight, SSD1306_WHITE);   
  }
#endif// MONO_OLED_128x64

#ifdef st7735_tft

   if (sprSel){
   spr[sprSel].fillRectHGradient(0, 0, 128, 160, TFT_RED, TFT_BLUE);
 
//   spr[sprSel].fillScreen(TFT_BLACK); // clear screen on framebuffer 
   } else {
   spr[sprSel].fillScreen(TFT_RED); // clear screen on framebuffer 
//   spr[sprSel].fillRectHGradient(0, 0, 128, 160, TFT_RED, TFT_BLUE);

//   spr[sprSel].fillScreen(TFT_BLACK); // clear screen on framebuffer 
   }

  // Draw the graph data as vertical bars
  for (uint8_t i = 0; i < (ROLLING_SIZE_TFT-1); i++) {
    // Map the data value to the graph height
    uint8_t barHeight = map(rollingBuffer_tft[i], graphMin_tft, graphMax_tft, 0, TFT_HEIGHT-1 );
    
    // Draw a vertical bar from the bottom to the data value
//    display.drawLine(graphX + i, graphY + graphH , graphX + i, graphY + graphH - barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY + graphH , barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY+(graphH-barHeight) , barHeight, SSD1306_WHITE);
      spr[sprSel].drawFastVLine( i, (TFT_HEIGHT-barHeight) , barHeight, TFT_GREEN);   
//    spr[sprSel].drawFastVLine( i, 0 , TFT_HEIGHT-barHeight, TFT_BLACK);   
//    tft.drawFastVLine( i, 0 , barHeight, TFT_BLUE);   

  }
#endif // st7735_tft

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

#ifdef GRAPH_TIMING // time accountig for debug/testing
#ifdef MONO_OLED_128x64
  display.fillRect(OLED_WIDTH-4*8,0,4*8,8,WHITE);
  display.setCursor(OLED_WIDTH-4*8,0);
  display.setTextColor(BLACK);
  display.print(micros()-graph_redraw_time);
  display.display();
#endif //MONO_OLED_128x64

#ifdef st7735_tft
//  tft.fillRect(TFT_WIDTH-4*8,0,4*8,8,ST7735_WHITE);
  spr[sprSel].fillRect(TFT_WIDTH-4*8,0,4*8,8,TFT_WHITE);
  spr[sprSel].setCursor(TFT_WIDTH-4*8,0);
//  tft.setTextColor(ST7735_RED);
  spr[sprSel].setTextColor(TFT_RED);
  spr[sprSel].print(micros()-graph_redraw_time);
#endif // st7735_tft
  
#endif// GRAPH_TIMING

#ifdef st7735_tft
  // release exclusive use of the SPI bus
  if (tft.dmaBusy()) {
//    digitalWrite(TFT_LED,HIGH); // disable screen during DMA transfer
   } else { 
//         digitalWrite(TFT_LED,LOW); // enable screen when DMA is idle
         tft.endWrite(); // release exclusive use of SPI bus
         }
//  tft.endWrite();
#endif // st7735_tft


}
