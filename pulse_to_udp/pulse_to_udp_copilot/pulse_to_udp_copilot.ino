// Dreamed by Copilot
// Define the pin where the pulse signal is connected
#define PULSE_PIN 4

// Define the maximum size of the arrays
#define MAX_SIZE 8

// Define the UDP broadcast parameters
#define UDP_IP "224.0.1.187"
#define UDP_PORT 5683

// Include the necessary libraries
//#include <Arduino.h>
//#include <WiFi.h>
//#include <WiFiUdp.h>

#include "WiFi.h"
#include "AsyncUDP.h"


// Declare a pragma packed struct to store the pulse data
#pragma pack(push, 1)
struct PulseData {
  uint32_t rpm[MAX_SIZE]; // Array to store the RPM values
  uint32_t length[MAX_SIZE]; // Array to store the pulse length values
  uint8_t index; // Index to keep track of the array position
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

// Declare a function to handle the pulse interrupt
void ICACHE_RAM_ATTR handlePulse();

// Setup function
void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);

  // Initialize the pulse pin as input
  pinMode(PULSE_PIN, INPUT);

  // Initialize the pulse data index to zero
  pulseData.index = 0;

  // Attach the pulse interrupt to the pulse pin
//  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulse, CHANGE);

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

  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulse, CHANGE);

  // Initialize the UDP object
//  udp.begin(UDP_PORT);
}

// Loop function
void loop() {
  // Check if the pulse data arrays are full
//    Serial.println(pulseData.index);
//    delay(100);
  if (pulseData.index == (MAX_SIZE-1)) {
    // Send the pulse data struct using UDP broadcast
//    udp.beginPacketMulticast(IPAddress(224, 0, 1, 187), UDP_PORT, WiFi.localIP());
    udp.connect(IPAddress(224, 0, 1, 187), UDP_PORT);
//      udp.broadcastTo((uint8_t *)&pulseData, sizeof(pulseData),UDP_PORT,IPAddress(224, 0, 1, 187));
    udp.write((uint8_t *)&pulseData, sizeof(pulseData));
    udp.close(); 
//    udp.endPacket();

//    Serial.println("Pulse data sent");

    // Reset the pulse data index to zero
    pulseData.index = 0;
  }
}

// Function to handle the pulse interrupt
void ICACHE_RAM_ATTR handlePulse() {
  // Get the current time in microseconds
  uint32_t currentTime = micros();

  // Check the state of the pulse pin
  if (digitalRead(PULSE_PIN) == HIGH) {
    // Rising edge detected
    // Calculate the time difference since the last pulse
    uint32_t timeDiff = currentTime - lastPulseTime;

    // Calculate the RPM value
    uint32_t rpm = 60000000 / timeDiff;

    // Store the RPM value in the array
    pulseData.rpm[pulseData.index] = rpm;

    // Update the last pulse time
    lastPulseTime = currentTime;
  } else {
    // Falling edge detected
    // Calculate the pulse length
    uint32_t pulseLength = currentTime - lastPulseTime;

    // Store the pulse length in the array
    pulseData.length[pulseData.index] = pulseLength;

    // Increment the pulse data index
    if (pulseData.index < (MAX_SIZE -1)) pulseData.index++;
  }
}
