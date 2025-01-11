
// Dreamed by copilot
#include "wifi_settings.h" // see to enable AP mode too
#include "display_settings.h" // include settings of the display 

//IR reciever includes - to select screen, setup display etc. 
#define USE_IR_INPUT // use IR remote to change stuff and somethings
#define USE_IR_INPUT_DEBUG // define to print all decoded keys to serial output

#ifdef USE_IR_INPUT 
#include "PinDefinitionsAndMore.h" // Sets input pin to 15 (on ESP32)

//#define NO_LED_FEEDBACK_CODE   // Activate this if you want to suppress LED feedback or if you do not have a LED. This saves 14 bytes code and 2 clock cycles per interrupt.

#define IRMP_PROTOCOL_NAMES 1 // Enable protocol number mapping to protocol strings - requires some FLASH.

//#define IRMP_SUPPORT_SIRCS_PROTOCOL      1
//#define IRMP_SUPPORT_NEC_PROTOCOL        1  // includes APPLE and ONKYO protocols
//#define IRMP_SUPPORT_SAMSUNG_PROTOCOL    1
//#define IRMP_SUPPORT_KASEIKYO_PROTOCOL   1

//#define IRMP_SUPPORT_JVC_PROTOCOL        1
//#define IRMP_SUPPORT_NEC16_PROTOCOL      1
//#define IRMP_SUPPORT_NEC42_PROTOCOL      1
//#define IRMP_SUPPORT_MATSUSHITA_PROTOCOL 1
//#define IRMP_SUPPORT_DENON_PROTOCOL      1
//#define IRMP_SUPPORT_RC5_PROTOCOL        1
//#define IRMP_SUPPORT_RC6_PROTOCOL        1
//#define IRMP_SUPPORT_IR61_PROTOCOL       1
#define IRMP_SUPPORT_GRUNDIG_PROTOCOL    1
//#define IRMP_SUPPORT_SIEMENS_PROTOCOL    1
//#define IRMP_SUPPORT_NOKIA_PROTOCOL      1

/*
 * After setting the definitions we can include the code and compile it.
 */
#include <irmp.hpp>

IRMP_DATA irmp_data;

#endif //#ifdef USE_IR_INPUT

//end of IR related includes
//fixme - separate .h , #defines etc. 

#define TRANSLATION_TIMEBASE 24000.0 // timebase for translations from pulse count to ms
                                   // translation to ms is done in float realm. 

//#define HANDLE_PACKET_SERIAL_DEBUG  // prints core ID and time spent on packet processing
// about 2-3 ms per packet. runs on core 0 . 

#define ACCOUNTING_CONSUMPTION // store injection pulses for consuption integration accounting
#ifdef ACCOUNTING_CONSUMPTION
#define ROLLING_SIZE_CONSUMPTION 100*60// 100 Hz = 100 data points per second. 
                                        // *60 = minute, 6000 points, 4 bytes *2 (lenght and rpm) = 48000 bytes                                        
//consumption = consumption *INJECTOR_FLOW_RATE*FUEL_DENSITY
#define INJECTOR_FLOW_RATE 10.0 // L/sec
#define FUEL_DENSITY 0.75       // g/L
#define INTEGRATION_PERIOD 1000 // ms
uint32_t last_integration_time = 0 ; // variable to store last integration time
float sec_integrated_consumption = 0 ; // variable to store last 1sec integrated fuel consumption
bool  sec_new_integration = false ; // flag to indicate new integration is ready
#endif // #ifdef ACCOUNTING_CONSUMPTION

#define GRAPH_LINE // about 3ms 
#define GRAPH_BARS_CONSUMPTION
//#define GRAPH_DOTS
#define GRAPH_BARS // about 3-4ms 
//#define GRAPH_TEST // test the display without incoming packets by initalizing with random data
#define GRAPH_TIMING // draw time needed to display redraw 

uint8_t desktop_number = 0 ; // variable to set desktop number as set by IR remote

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

#ifdef ACCOUNTING_CONSUMPTION
float rollingBuffer_consumption_inj[ROLLING_SIZE_CONSUMPTION];
float rollingBuffer_consumption_rpm[ROLLING_SIZE_CONSUMPTION];
float accountingMin_inj = 0 ;
float accountingMax_inj = 0 ;
float accountingMin_rpm = 0 ; 
float accountingMax_rpm = 0 ;


#ifdef GRAPH_BARS_CONSUMPTION
#ifdef st7735_tft
float consumption_rollingBuffer_tft[ROLLING_SIZE_TFT];
float consumption_graphMin_tft = 0;
float consumption_graphMax_tft = 0;
// Declare a global variable to store the rolling buffer index
//uint8_t rollingIndex_tft = 0;
#endif// st7735_tft
#endif //#ifdef GRAPH_BARS_CONSUMPTION


#endif //#ifdef ACCOUNTING_CONSUMPTION


#ifdef GRAPH_TIMING
uint32_t graph_redraw_time = 0; 
#endif// GRAPH_TIMING

bool new_packet = 0 ; // new packet flag

// Declare a function to handle the UDP packet
IRAM_ATTR void handlePacket(AsyncUDPPacket packet);

// Declare a function to draw the line graph on the OLED display
void drawGraph();

/*
void UDP_injection1( void * parameter) {
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

  delay(100);
    // Set the callback function to handle the UDP packet
    udp.onPacket(handlePacket);
  }

for (;;) {
    // do something interesting
    //Serial.println("hey, this is core 0");
    // Add a small delay to let the watchdog process
    //https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    delay(1000);
}
  
}

TaskHandle_t Task0;

*/

 // does not work at all 

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

//xTaskCreatePinnedToCore(
//      UDP_injection1, /* Function to implement the task - asyncUDP recieving injection data */
//      "UDP_injection", /* Name of the task */
//      5000,  /* Stack size in words */
//      NULL,  /* Task input parameter */
//      0,  /* Priority of the task */
//      &Task0,  /* Task handle. */
//      1); /* Core where the task should run */

delay(200);

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


// IR
#ifdef USE_IR_INPUT
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRMP));

    irmp_init();

    Serial.print(F("Ready to receive IR signals of protocols: "));
    irmp_print_active_protocols(&Serial);
    Serial.println(F("at pin " STR(IRMP_INPUT_PIN)));

//    irmp_irsnd_LEDFeedback(true); // Enable receive signal feedback at ALTERNATIVE_IR_FEEDBACK_LED_PIN

#ifdef MONO_OLED_128x64
//  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 16+8);
  display.print("IR: ");
  display.println(irmp_print_active_protocols(&Serial));
  display.display();
#endif// MONO_OLED_128x64

#ifdef st7735_tft
  tft.startWrite();
  tft.setTextSize(1);
//  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(0, 16+8);
  tft.print("IR: ");
//  tft.setTextColor(ST7735_BLUE,ST7735_BLACK);
  tft.setTextColor(TFT_BLUE|TFT_GREEN,TFT_BLACK);
  tft.print("GRUNDIG "); // fixme - hardcoded

//  tft.println(irmp_print_active_protocols(&tft.println));
//  irmp_print_active_protocols(&tft.print);

  tft.endWrite();

#endif// st7735_tft

#endif // #ifdef USE_IR_INPUT

#ifdef MONO_OLED_128x64
//  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 16+8+8);
  display.print("ready: ");
  display.println(millis());
  display.display();
#endif// MONO_OLED_128x64

#ifdef st7735_tft
  tft.startWrite();
  tft.setTextSize(1);
//  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(0, 16+8+8);
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

#ifdef USE_IR_INPUT

void handle_IR() {
    /*
     * Check if new data available and get them
     */
    if (irmp_get_data(&irmp_data))
    {
        /*
         * Here data is available -> evaluate IR command
         */
        switch (irmp_data.command)
        {
        case 0x10:
        desktop_number = 0x00; 
        break; 
        case 0x11:
        desktop_number = 0x01; 
        break; 
        case 0x12:
        desktop_number = 0x02; 
        break; 
        case 0x13:
        desktop_number = 0x03; 
        break; 

          
        case 0x0:
            Serial.println(F("OK"));
            break;
        default:
            break;
        }
#ifdef USE_IR_INPUT_DEBUG
        irmp_result_print(&irmp_data);
#endif // #ifdef USE_IR_INPUT_DEBUG
        
    }
}
#endif // #ifdef USE_IR_INPUT 


// Loop function
void loop() {
#ifdef USE_IR_INPUT 
  handle_IR();
#endif // #ifdef USE_IR_INPUT 


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

// A function to integrate pulses representing fuel injection impulses
// The buffer array consists of two parts: pulse lengths and rpm values
// The function reports integrated fuel consumption over a time window
// The function infers spacing of the pulses from rpm values

// Parameters:
// buffer: a two-dimensional array of pulse lengths and rpm values
//(using globally declared buffer)

// size: the size of the buffer array
//(using globally defined max buffer size)

// window: the time window in seconds
// Returns: the integrated fuel consumption in milliliters

// THE FUNCTION INTEGRATES BACKWARDS (from the end to the beginning of the buffer)
// the ROLLING_SIZE_CONSUMPTION buffer must be big enough to hold all the pulses occuring during time window of integration
// at the predicted max pulse rate (rpm)

//float rollingBuffer_consumption_inj[ROLLING_SIZE_CONSUMPTION];

//float integratePulses(float buffer[][2], int size, float window) {
float integratePulses(uint16_t window) {

  // Initialize variables
  float consumption = 0; // the integrated fuel consumption
  float time = 0; // the current time
//  float time = last_time_leftover; // the current time
  float interval = 0; // the time interval between pulses
  float volume = 0; // the volume of fuel injected per pulse
//  float density = 0.75; // the density of fuel in g/ml
  uint16_t i = ROLLING_SIZE_CONSUMPTION-1; // the index of the buffer array

  // Loop through the buffer array
//  while (i < size && time < window) {
  while (i > 1 && time < window) {

    // Calculate the time interval between pulses from rpm values
    // rpm = 60 / (interval * 2)
    // interval = 30 / rpm
//    interval = 30 / buffer[i][1];
    interval = 60000.0 / rollingBuffer_consumption_rpm[i];

    // Calculate the volume of fuel injected per pulse from pulse lengths
    // volume = pulse length * flow rate
    // Assume a constant flow rate of 10 ml/s
//    volume = buffer[i][0] * 10;
//    volume = rollingBuffer_consumption_inj[i] * INJECTOR_FLOW_RATE;
    volume = rollingBuffer_consumption_inj[i] ; // do not accumulate error 

    // Update the integrated fuel consumption
    // consumption = consumption + volume * density
    consumption = consumption + volume; // do not accumulate error

    // Update the current time
    // time = time + interval
    time = time + interval;
//    Serial.println (time);
    // Increment the index
//    i++;
    // Decrement the index
    i--;

  }
//  consumption = consumption *INJECTOR_FLOW_RATE*FUEL_DENSITY; // move this outside handlepacket to speed things up. 

  // Return the integrated fuel consumption
  return consumption;
}

// Function to handle the UDP packet
IRAM_ATTR void handlePacket(AsyncUDPPacket packet) {
  // Declare a local variable to store the pulse data
//  PulseData pulseData;
#ifdef HANDLE_PACKET_SERIAL_DEBUG
  uint32_t packet_processing_start_time = micros(); 
  Serial.print("UDP_injection1 on core ");
  Serial.println(xPortGetCoreID());
#endif //#ifdef HANDLE_PACKET_SERIAL_DEBUG
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

#ifdef ACCOUNTING_CONSUMPTION
//float rollingBuffer_consumption_inj[ROLLING_SIZE_CONSUMPTION];
//float rollingBuffer_consumption_rpm[ROLLING_SIZE_CONSUMPTION];

  for (uint16_t i = 0; i < PACKET_BUFFER_SIZE; i++) {

    //Roll the rolling buffer.
    // Shift the graph data to the left by one pixel and find the maximum value
  accountingMin_inj = accountingMax_inj; // set graphMin to last graphMax value
  accountingMax_inj = 0;
  accountingMin_rpm = accountingMax_rpm; // set graphMin to last graphMax value
  accountingMax_rpm = 0;
  
  for (uint16_t i = 0; i < (ROLLING_SIZE_CONSUMPTION - 1); i++) {

    rollingBuffer_consumption_inj[i] = rollingBuffer_consumption_inj[i + 1];
    rollingBuffer_consumption_rpm[i] = rollingBuffer_consumption_rpm[i + 1];
    
    if (rollingBuffer_consumption_inj[i] > accountingMax_inj) {
      accountingMax_inj = rollingBuffer_consumption_inj[i];
      }
    if (rollingBuffer_consumption_inj[i] < accountingMin_inj) {
      accountingMin_inj = rollingBuffer_consumption_inj[i];
    }

    if (rollingBuffer_consumption_rpm[i] > accountingMax_rpm) {
      accountingMax_rpm = rollingBuffer_consumption_rpm[i];
      }
    if (rollingBuffer_consumption_rpm[i] < accountingMin_rpm) {
      accountingMin_rpm = rollingBuffer_consumption_rpm[i];
    }
    
    // graphMax_tft = max(graphMax_tft, rollingBuffer_tft[i]); 
    // or use that instead 
  }
    // Add the pulse length to the rolling buffer    
//    rollingBuffer_consumption_inj[ROLLING_SIZE_CONSUMPTION-1] = pulseData.length[i];
    rollingBuffer_consumption_inj[ROLLING_SIZE_CONSUMPTION-1] = pulseData.length[i]/TRANSLATION_TIMEBASE; // convert to ms
                                                                                              // this saves time on integration pass
    rollingBuffer_consumption_rpm[ROLLING_SIZE_CONSUMPTION-1] = pulseData.rpm[i]; // rpm is in float already. 

    // Increment the rolling buffer index and wrap around if needed
    //rollingIndex_tft = (rollingIndex_tft + 1) % ROLLING_SIZE;
  }

//#define INTEGRATION_PERIOD 1000 // ms
//uint32_t last_integration_time = 0 ; // variable to store last integration time

  if (millis() - last_integration_time >= INTEGRATION_PERIOD ) {
    last_integration_time = millis();   
    sec_integrated_consumption = integratePulses(INTEGRATION_PERIOD); 

#ifdef GRAPH_BARS_CONSUMPTION
#ifdef st7735_tft
//uint32_t consumption_buffer_tft[ROLLING_SIZE_TFT];
//uint32_t consumption_graphMin_tft = 0;
//uint32_t consumption_graphMax_tft = 0;
// Declare a global variable to store the rolling buffer index
//uint8_t rollingIndex_tft = 0;

  consumption_graphMin_tft = consumption_graphMax_tft; // set graphMin to last graphMax value
  consumption_graphMax_tft = 0;
  for (int i = 0; i < (ROLLING_SIZE_TFT - 1); i++) {

    consumption_rollingBuffer_tft[i] = consumption_rollingBuffer_tft[i + 1];
    if (consumption_rollingBuffer_tft[i] > consumption_graphMax_tft) {
      consumption_graphMax_tft = consumption_rollingBuffer_tft[i];
      }
    if (consumption_rollingBuffer_tft[i] < consumption_graphMin_tft) {
      consumption_graphMin_tft = consumption_rollingBuffer_tft[i];
    }
    // consumption_graphMax_tft = max(consumption_graphMax_tft, consumption_rollingBuffer_tft[i]); 
    // or use that instead 
  }
    // Add the pulse length to the rolling buffer    
    consumption_rollingBuffer_tft[ROLLING_SIZE_TFT-1] = sec_integrated_consumption;  

#endif// st7735_tft
#endif //#ifdef GRAPH_BARS_CONSUMPTION

    sec_new_integration = true; 
  }

#endif //#ifdef ACCOUNTING_CONSUMPTION

  new_packet = true; 

#ifdef HANDLE_PACKET_SERIAL_DEBUG
  //uint32_t packet_processing_start_time = micros(); 
//  Serial.print("micros used: ");
  Serial.println(micros()-packet_processing_start_time);
#endif //#ifdef HANDLE_PACKET_SERIAL_DEBUG

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

#ifdef st7735_tft
if (desktop_number==0x00) {

  spr[sprSel].fillRect(0,TFT_HEIGHT-8,6*8,8,TFT_BLACK);
  spr[sprSel].setCursor(0, TFT_HEIGHT-8);
  
  if (pulseData.rpm[PACKET_BUFFER_SIZE-1] < 1000 ){ 
    spr[sprSel].setTextColor(TFT_WHITE);}
  if (pulseData.rpm[PACKET_BUFFER_SIZE-1] > 1000 ){ 
    spr[sprSel].setTextColor(TFT_GREEN);}
  if (pulseData.rpm[PACKET_BUFFER_SIZE-1] > 1200 ){ 
    spr[sprSel].setTextColor(TFT_YELLOW);}
  if (pulseData.rpm[PACKET_BUFFER_SIZE-1] > 1800 ){ 
    spr[sprSel].setTextColor(0x010f0f);}
  if (pulseData.rpm[PACKET_BUFFER_SIZE-1] > 3000 ){ 
    spr[sprSel].setTextColor(TFT_YELLOW);}
  if (pulseData.rpm[PACKET_BUFFER_SIZE-1] > 4000 ){ 
    spr[sprSel].setTextColor(TFT_RED);}
    
  spr[sprSel].print(pulseData.rpm[PACKET_BUFFER_SIZE-1]);

//  display.print(pulseData.rpm[1]);
//  display.setCursor(6*8, 0);
//  display.print(pulseData.index);

  spr[sprSel].fillRect(0,TFT_HEIGHT-8-8,6*8,8,TFT_BLACK);
  spr[sprSel].setCursor(0, TFT_HEIGHT-8-8);
  spr[sprSel].setTextColor(TFT_WHITE);
  spr[sprSel].print(pulseData.length[PACKET_BUFFER_SIZE-1]);

  spr[sprSel].fillRect(0+7*8,TFT_HEIGHT-8-8,8*8,8,TFT_BLACK);
  spr[sprSel].setCursor(0+7*8, TFT_HEIGHT-8-8);
  spr[sprSel].setTextColor(TFT_WHITE);
//  spr[sprSel].print(pulseData.length[PACKET_BUFFER_SIZE-1]);
  spr[sprSel].print(rollingBuffer_consumption_inj[ROLLING_SIZE_CONSUMPTION-1],4);

  spr[sprSel].fillRect(0+7*8,TFT_HEIGHT-8-8+8,8*8,8,TFT_BLACK);
  spr[sprSel].setCursor(0+7*8, TFT_HEIGHT-8-8+8);
  spr[sprSel].setTextColor(TFT_YELLOW);
//  spr[sprSel].print(pulseData.length[PACKET_BUFFER_SIZE-1]);
  spr[sprSel].print(sec_integrated_consumption,4);

//float sec_integrated_consumption = 0 ; // variable to store last 1sec integrated fuel consumption
//bool  sec_new_integration = false ; // flag to indicate new integration is ready

//float rollingBuffer_consumption_inj[ROLLING_SIZE_CONSUMPTION];


}

#endif st7735_tft


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
if (desktop_number==0x01) {
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


#ifdef st7735_tft
if (desktop_number==0x02) {

   if (sprSel){
//   spr[sprSel].fillRectHGradient(0, 0, 128, 160, TFT_RED, TFT_BLUE); 
   spr[sprSel].fillScreen(TFT_BLACK); // clear screen on framebuffer 
   } else {
//   spr[sprSel].fillScreen(TFT_RED); // clear screen on framebuffer 
//   spr[sprSel].fillRectHGradient(0, 0, 128, 160, TFT_RED, TFT_BLUE);
   spr[sprSel].fillScreen(TFT_BLACK); // clear screen on framebuffer 
   }

  // Draw the graph data as vertical bars
  for (uint8_t i = 0; i < (ROLLING_SIZE_TFT-1); i++) {
    // Map the data value to the graph height
    uint8_t barHeight = map(consumption_rollingBuffer_tft[i], consumption_graphMin_tft, consumption_graphMax_tft, 0, TFT_HEIGHT-1 );
    
    // Draw a vertical bar from the bottom to the data value
//    display.drawLine(graphX + i, graphY + graphH , graphX + i, graphY + graphH - barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY + graphH , barHeight, SSD1306_WHITE);
    //display.drawFastVLine(graphX + i, graphY+(graphH-barHeight) , barHeight, SSD1306_WHITE);    
      spr[sprSel].drawFastVLine( i, (TFT_HEIGHT-barHeight) , barHeight, TFT_GREEN);   
//    spr[sprSel].drawFastVLine( i, 0 , TFT_HEIGHT-barHeight, TFT_BLACK);   
//    tft.drawFastVLine( i, 0 , barHeight, TFT_BLUE);   

  }

//  display.fillRect(0,0,5*8,8,BLACK);
  spr[sprSel].setCursor(0, 0);
  spr[sprSel].setTextColor(TFT_WHITE,TFT_BLACK);
  spr[sprSel].print(consumption_graphMax_tft,3);
//  display.fillRect(0,OLED_HEIGHT-8,5*8,8,BLACK);
  spr[sprSel].setCursor(0, TFT_HEIGHT-8);
  spr[sprSel].print(consumption_graphMin_tft,3);
  spr[sprSel].setCursor(6*8, 0);
  spr[sprSel].print(rollingBuffer_consumption_inj[ROLLING_SIZE_CONSUMPTION-1],4);

 // spr[sprSel].print(pulseData.length[PACKET_BUFFER_SIZE-1]);

//  spr[sprSel].fillRect(0+7*8,TFT_HEIGHT-8-8,8*8,8,TFT_BLACK);
//  spr[sprSel].setCursor(0+7*8, TFT_HEIGHT-8-8);
//  spr[sprSel].setTextColor(TFT_WHITE);
//  spr[sprSel].print(pulseData.length[PACKET_BUFFER_SIZE-1]);
//  spr[sprSel].print(rollingBuffer_consumption_inj[ROLLING_SIZE_CONSUMPTION-1],4);

//  spr[sprSel].fillRect(0+7*8,TFT_HEIGHT-8-8+8,8*8,8,TFT_BLACK);
  spr[sprSel].setCursor(0+7*8, TFT_HEIGHT-8-8+8);
  spr[sprSel].setTextColor(TFT_YELLOW,TFT_BLACK);
//  spr[sprSel].print(pulseData.length[PACKET_BUFFER_SIZE-1]);
  spr[sprSel].print(sec_integrated_consumption,4);


}
#endif // st7735_tft

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
