//Dreamed by copilot
// Define the UDP parameters
#define UDP_IP "224.0.1.187"
#define UDP_PORT 5683

#define MAX_SIZE 100 //Max size of buffers

// Include the necessary libraries
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>

// Declare a pragma packed struct to store the pulse data
#pragma pack(push, 1)
struct PulseData {
  uint32_t rpm[MAX_SIZE]; // Array to store the RPM values
  uint32_t length[MAX_SIZE]; // Array to store the pulse length values
  uint8_t index; // packet index. 
};
#pragma pack(pop)

// Declare a global variable to store the asyncUDP object
AsyncUDP udp;

// Declare a global variable to store the pulse data
PulseData pulseData;

// Declare a function to handle the UDP packet
void handlePacket(AsyncUDPPacket packet);

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
}

// Loop function
void loop() {
  // Do nothing
}

// Function to handle the UDP packet
void handlePacket(AsyncUDPPacket packet) {
  // Copy the UDP packet data to the pulse data variable
  memcpy(&pulseData, packet.data(), sizeof(pulseData));

  // Print the pulse data to the serial monitor
  Serial.println("Pulse data received");
  Serial.print("index: ");
  Serial.println(pulseData.index);
  for (int i = 0; i < MAX_SIZE; i++) {
    Serial.print("RPM: ");
    Serial.print(pulseData.rpm[i]);
    Serial.print(" Length: ");
    Serial.println(pulseData.length[i]);
  }
}
