//Nightmare by Gemini
#include <WiFi.h>
#include <AsyncUDP.h>
#include <Adafruit_SSD1306.h>

// Define WIFI credentials
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";

// Define UDP port and server address
const int port = 5683;
const char* server_address = "224.0.1.187";

// Define buffer sizes and array size in data packet
const int packet_buffer_size = sizeof(DataPacket);
const int data_buffer_size = 256;
const int pulse_length_array_size = 50;

// Define OLED display characteristics
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);

// Struct for data (packed to byte aligned)
#pragma pack(push, 1)
struct DataPacket {
  uint32_t frequencies[pulse_length_array_size];
  uint32_t pulseLengths[pulse_length_array_size];
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
    
    // Find min and max pulse lengths within the latest packet
    uint32_t min_pulse = data_buffer[0].pulseLengths[0];
    uint32_t max_pulse = data_buffer[0].pulseLengths[0];
    for (int i = 1; i < pulse_length_array_size; i++) {
      if (data_buffer[0].pulseLengths[i]) { // Skip missing data
        min_pulse = min(min_pulse, data_buffer[0].pulseLengths[i]);
        max_pulse = max(max_pulse, data_buffer[0].pulseLengths[i]);
      }
    }
    
    // Map pulse lengths to screen height using dynamic scaling
    int data_index = data_buffer_index * pulse_length_array_size; // Calculate starting index based on data buffer index
    for (int i = 0; i < SCREEN_WIDTH; i++) {
      // Calculate offset within the pulse length array
      int offset = (i * pulse_length_array_size / SCREEN_WIDTH) % pulse_length_array_size;
      
      // Get pulse length from the latest data using data_index and offset
      uint32_t pulse_length = data_buffer[0].pulseLengths[data_index + offset];
      
      // Check if data is available and skip missing data
      if (data_buffer[0].pulseLengths[data_index + offset]) {
        int height = map(pulse_length, min_pulse, max_pulse, 0, SCREEN_HEIGHT - 1);
        display.drawLine(i, SCREEN_HEIGHT - 1, i, SCREEN_HEIGHT - 1 - height);
      }
    }
    
    display.display();
    last_update = millis();
  }
}

// Main loop
void loop() {
  // Add your custom logic here, e.g., user interaction, network communication, etc.
  
  // Update display regularly
  draw_graphics();
}

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
