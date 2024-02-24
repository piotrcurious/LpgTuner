//Dreamed by copilot 

// Define the UDP parameters
#define UDP_IP "224.0.1.187"
#define UDP_PORT 5683

// Include the necessary libraries
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Declare a pragma packed struct to store the pulse data
#pragma pack(push, 1)
struct PulseData {
  uint32_t rpm[100]; // Array to store the RPM values
  uint32_t length[100]; // Array to store the pulse length values
  uint8_t index; // Index to keep track of the array position
};
#pragma pack(pop)

// Declare a global variable to store the UDP object
WiFiUDP udp;

// Declare a global variable to store the pulse data
PulseData pulseData;

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

  // Initialize the UDP object
  udp.beginMulticast(WiFi.localIP(), IPAddress(224, 0, 1, 187), UDP_PORT);
}

// Loop function
void loop() {
  // Check if there is any UDP packet available
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the UDP packet and store it in the pulse data variable
    udp.read((uint8_t *)&pulseData, sizeof(pulseData));

    // Print the pulse data to the serial monitor
    Serial.println("Pulse data received");
    for (int i = 0; i < pulseData.index; i++) {
      Serial.print("RPM: ");
      Serial.print(pulseData.rpm[i]);
      Serial.print(" Length: ");
      Serial.println(pulseData.length[i]);
    }
  }
}
