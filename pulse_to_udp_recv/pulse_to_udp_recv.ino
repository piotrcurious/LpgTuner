// UDP Pulse Data Receiver (ESP32)
// Receives multicast UDP packets containing batch pulse data (RPM and Length).

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// UDP parameters
const char* ssid = "your-ssid";
const char* password = "your-password";
const char* udpAddress = "224.0.1.187";
const int udpPort = 5683;

// Data structure
#pragma pack(push, 1)
struct PulseData {
  uint32_t rpm[100];
  uint32_t length[100];
  uint8_t count;
};
#pragma pack(pop)

WiFiUDP udp;
PulseData pulseData;

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");

  // Join multicast group
  udp.beginMulticast(IPAddress(224, 0, 1, 187), udpPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize == sizeof(PulseData)) {
    int readLen = udp.read((uint8_t *)&pulseData, sizeof(pulseData));

    if (readLen == sizeof(PulseData)) {
        Serial.printf("Received batch of %d pulses\n", pulseData.count);
        int limit = (pulseData.count > 100) ? 100 : pulseData.count;
        for (int i = 0; i < limit; i++) {
          Serial.printf("[%d] RPM: %u, Length: %u us\n", i, pulseData.rpm[i], pulseData.length[i]);
        }
    }
  } else if (packetSize > 0) {
      // Discard malformed packet
      while(udp.available()) udp.read();
  }
}
