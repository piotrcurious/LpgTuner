//Dreamed by Gemini 
#include <WiFi.h>
#include <WiFiUdp.h>

// Define buffer sizes
const int bufferSize = 50;

// Pin for pulse input
const int pulsePin = 14;

// Struct for data (packed to byte aligned)
#pragma pack(push, 1)
struct DataPacket {
  uint32_t frequencies[bufferSize];
  uint32_t pulseLengths[bufferSize];
  uint8_t index;
} __attribute__((packed));
#pragma pack(pop)

DataPacket dataPacket;

// Global variables for interrupt functions
volatile unsigned long lastPulseStart = 0;
volatile unsigned long pulseStart = 0;
volatile bool pulseStarted = false;

// Interrupt service routine for pulse start
void IRAM_ATTR pulseStartISR() {
  pulseStart = micros();
  pulseStarted = true;
}

// Interrupt service routine for pulse end
void IRAM_ATTR pulseEndISR() {
  if (pulseStarted) {
    unsigned long pulseLength = micros() - pulseStart;

    // Calculate frequency based on current and last pulse start
   // if (dataPacket.index > 0) { it remains continuous 
      dataPacket.pulseLengths[dataPacket.index] = pulseLength;
      dataPacket.frequencies[dataPacket.index] = 1000000L / (pulseStart - lastPulseStart);
      dataPacket.index++;
    //}

    lastPulseStart = pulseStart;
    pulseStarted = false;

    // Check if arrays are full and trigger data sending in loop
    if (dataPacket.index == bufferSize) {
      dataSendingFlag = true;
    }
  }
}

// Flag for data sending in loop (avoids sending within ISR)
volatile bool dataSendingFlag = false;

// Function to send data via UDP multicast
void sendData() {
  WiFiUDP udp;
  if (udp.beginPacketMulticast("224.0.1.187", 5683)) {
    udp.write((uint8_t*)&dataPacket, sizeof(dataPacket));
    udp.endPacket();
  }
  dataPacket.index = 0; // Reset index after sending
  dataSendingFlag = false;
}

void setup() {
  Serial.begin(115200);

  // Initialize WiFi (if needed for UDP)
  // Add WiFi setup if required

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(pulsePin), pulseStartISR, RISING);
  attachInterrupt(digitalPinToInterrupt(pulsePin), pulseEndISR, FALLING);
}

void loop() {
  if (dataSendingFlag) {
    sendData();
  }

  // Other tasks in loop
}
