// Dreamed by Copilot

#include "wifi_settings.h" // see to enable AP mode too

// Define the pin where the pulse signal is connected
#define PULSE_PIN 4
//#define PULSE_LOGIC_POLARITY LOW // define if pulse is logical high or logical low. 
#define PULSE_LOGIC_POLARITY HIGH // define if pulse is logical high or logical low. 

// Define the maximum buffer size of the arrays
#define MAX_SIZE 2

#define TRANSLATION_TIMEBASE 240000000 // timebase for pulse count to RPM translation.
                                       // translation is done into float realm

// Declare a pragma packed struct to store the pulse data
#pragma pack(push, 1)
struct PulseData {
//  uint32_t rpm[MAX_SIZE]; // Array to store the RPM values
  float rpm[MAX_SIZE]; // Array to store the RPM values
  uint32_t length[MAX_SIZE]; // Array to store the pulse length values
  uint8_t index; // Index of packets
};
#pragma pack(pop)

// Declare a global variable to store the pulse data
PulseData pulseData;

// Declare a global variable to store the UDP object
//WiFiUDP udp;
AsyncUDP udp; 

// Declare a global variable to store the last pulse time
volatile uint32_t lastPulseTime = 0;

// Declare a global variable to store the last pulse length
volatile uint32_t lastPulseLength = 0;

// flag indicating last falling edge which could fit into buffer got recorded
bool data_ready = false ; 

uint8_t array_index = 0 ; // internal array pointer

// Declare a function to handle the pulse interrupt
void ICACHE_RAM_ATTR handlePulse();

// Setup function
void setup() {
  // Initialize the serial monitor
//  Serial.begin(115200);

  // Initialize the pulse pin as input
  pinMode(PULSE_PIN, INPUT);

  // Initialize the pulse data index to zero
  array_index = 0;

  // Attach the pulse interrupt to the pulse pin
//  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulse, CHANGE);
// later to not crash network 

/*
  // Connect to the WiFi network
  WiFi.begin("voltage", "irrolling12");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
*/

//  WiFi.persistent(false); // no need to wear off the flash as we have all data in the sketch

#ifdef AP_mode_on // if ap mode , start and configure AP
  WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
//  display.setTextSize(1);
//  display.setTextColor(WHITE);
//  display.setCursor(0, 8);
//  display.print("WiFi: ");
//  display.println(millis());
//  display.display();
//  delay(500);
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
//  display.setTextSize(1);
//  display.setTextColor(WHITE);
//  display.setCursor(0, 8);
//  display.print("WiFi: ");
//  display.println(millis());
//  display.display();
  delay(200);
//    Serial.println("Connecting to WiFi...");
    }
//  display.clearDisplay();
//  display.display();   
#endif AP_mode_on 

esp_wifi_set_ps(WIFI_PS_NONE);

  // Attach the pulse interrupt to the pulse pin
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulse, CHANGE);

  // Initialize the UDP object
//  udp.begin(UDP_PORT);
//    udp.connect(multicastIP, multicastPort);

}

void ICACHE_RAM_ATTR send_packet(){
    // Send the pulse data struct using UDP broadcast
//    udp.beginPacketMulticast(IPAddress(224, 0, 1, 187), UDP_PORT, WiFi.localIP());
//    udp.connect(IPAddress(224, 0, 1, 187), UDP_PORT);
    udp.connect(multicastIP, multicastPort);
//      udp.broadcastTo((uint8_t *)&pulseData, sizeof(pulseData),UDP_PORT,IPAddress(224, 0, 1, 187));
    udp.write((uint8_t *)&pulseData, sizeof(pulseData));
    // Reset the pulse data index to zero
    array_index = 0;
    data_ready = false; // reset data ready flag early to allow catching impulses before exit from send_packet
    udp.close(); 
//    udp.endPacket();

//    Serial.println("Pulse data sent");
    // Reset the pulse data index to zero
//    array_index = 0;
    pulseData.index++ ; // increment the packet index
//    data_ready = false; // reset data ready flag 
}

// Loop function
void loop() {
  if (data_ready) {
  send_packet(); // send packet 
  }  else {yield();}
}

// Function to handle the pulse interrupt
uint32_t currentTime ;

void ICACHE_RAM_ATTR handlePulse() {
// Get the current time in microseconds
//  uint32_t currentTime = micros();

// Get the current time in ESP32 clock cycles
//  uint32_t currentTime = ESP.getCycleCount();

// Get the current time in ESP32 clock cycles
// by using inline asm to avoid language and system overhead
  asm volatile("esync; rsr %0,ccount":"=a"(currentTime));

  // Check the state of the pulse pin
  if (digitalRead(PULSE_PIN) == PULSE_LOGIC_POLARITY) {
    // Rising edge detected
    // Calculate the time difference since the last pulse
    uint32_t timeDiff = currentTime - lastPulseTime;

    // Calculate the RPM value
//    uint32_t rpm = 60000000 / timeDiff; //60 000 000 / timeDiff in case of micros 
//    uint32_t rpm =(uint64_t) 60*240000000 / timeDiff; 
//    float rpm =(uint64_t) 60*240000000 / timeDiff;
//    float rpm =(float) 60*240000000 / timeDiff; //fixme - this should work but pauses for some reason

    // Store the RPM value in the array
//    pulseData.rpm[array_index] = rpm;

//    pulseData.rpm[array_index] = (float) 60*240000000 / timeDiff; //fixme - this should work but pauses for some reason,
                                                                    // probably when cycle counter overflows. 
//    pulseData.rpm[array_index] = (double) 60*240000000 / timeDiff; //fixme - this should work but pauses for some reason

    pulseData.rpm[array_index] = (double) 60*TRANSLATION_TIMEBASE / timeDiff; //fixme - this works only when done in double.. why?

    // Update the last pulse time
    lastPulseTime = currentTime;
  } else {
    // Falling edge detected
    // Calculate the pulse length
    uint32_t pulseLength = currentTime - lastPulseTime;

    // Store the pulse length in the array
    pulseData.length[array_index] = pulseLength;

    // Increment the pulse data index
    if (array_index < (MAX_SIZE -1)) {array_index++;} else { data_ready = true;} 
  }
}
