//nightmare by Gemini
#include <WiFi.h>
#include <AsyncUDP.h>
#include <Adafruit_SSD1306.h>

// Define WIFI credentials
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";

// Define UDP port and server address
const int port = 5683;
const char* server_address = "224.0.1.187";

// Define buffer sizes
const int packet_buffer_size = sizeof(DataPacket);
const int data_buffer_size = 256;

// Define OLED display characteristics
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);

// Struct for data (packed to byte aligned)
#pragma pack(push, 1)
struct DataPacket {
  uint32_t frequencies[50];
  uint32_t pulseLengths[50];
  uint8_t index;
} __attribute__((packed));
#pragma pack(pop)

// Data buffer for storing received packets
DataPacket data_buffer[data_buffer_size];

// Current index in data buffer
int data_buffer_index = 0;

// Last update time for graphics
unsigned long last_update = 0;

// Function to shift data buffer and store new packet
void store_data(DataPacket packet) {
  // Shift data buffer left
  for (int i = data_buffer_size - 1; i > 0; i--) {
    data_buffer[i] = data_buffer[i - 1];
  }
  
  // Copy new packet into last entry
  data_buffer[0] = packet;
}

// Function to draw pulse lengths on OLED display
void draw_graphics() {
  if (millis() - last_update >= 100) {
    display.clearDisplay();
    
    // Calculate scaling factor for pulse lengths
    float scale_factor = (float)SCREEN_HEIGHT / data_buffer[0].pulseLengths[0];
    
    // Draw pulse lengths
    for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
      int pulse_length = data_buffer[0].pulseLengths[i * 2];  // Get every other entry for smoother graph
      int height = pulse_length * scale_factor;
      display.drawLine(i, SCREEN_HEIGHT - 1, i + 1, SCREEN_HEIGHT - 1 - height);
    }
    
    display.display();
    last_update = millis();
  }
}

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SDA_PIN, SCL_PIN)) {
    Serial.println("SSD1306 Allocation failed");
  }
  display.clearDisplay();

  // Start AsyncUDP server
  AsyncUDP udp;
  udp.onPacketReceived([](AsyncUDPPacket pkt) {
    if (pkt.len() == packet_buffer_size) {
      DataPacket received_packet;
      memcpy(&received_packet, pkt.data(), packet_buffer_size);
      store_data(received_packet);
    }
  });
  udp.beginMulticast(port, server_address);
}

void loop() {
  draw_graphics();
}
