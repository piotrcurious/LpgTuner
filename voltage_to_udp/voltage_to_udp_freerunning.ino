
// Modified for analog sampling instead of pulse detection

#include "wifi_settings.h" // see to enable AP mode too

// Define the analog input pin
#define ANALOG_PIN 36  // ADC1_CH0 on ESP32
#define POT_PIN 39     // ADC1_CH3 - Potentiometer for sample rate control

// Define the maximum buffer size of the arrays
#define MAX_SIZE 2

// Sample rate range in milliseconds
#define MIN_SAMPLE_RATE_MS 1
#define MAX_SAMPLE_RATE_MS 20

// Declare a pragma packed struct to store the analog data
#pragma pack(push, 1)
struct AnalogData {
  float voltage[MAX_SIZE];     // Array to store voltage values in mV
  uint32_t timestamp[MAX_SIZE]; // Array to store timestamps
  uint8_t index;                // Index of packets
  uint32_t sample_rate_us;      // Current sample rate in microseconds
};
#pragma pack(pop)

// Declare a global variable to store the analog data
AnalogData analogData;

// Declare a global variable to store the UDP object
AsyncUDP udp; 

// flag indicating buffer is full and ready to send
bool data_ready = false; 

uint8_t array_index = 0; // internal array pointer

// Timer variables
hw_timer_t *timer = NULL;
volatile uint32_t sample_rate_us = 10000; // Default 10ms in microseconds

// Timer interrupt handler
void IRAM_ATTR onTimer() {
  // Read analog value in millivolts
  float voltage = analogReadMilliVolts(ANALOG_PIN);
  
  // Get current timestamp
  uint32_t currentTime = micros();
  
  // Store the voltage and timestamp
  analogData.voltage[array_index] = voltage;
  analogData.timestamp[array_index] = currentTime;
  
  // Increment the array index
  if (array_index < (MAX_SIZE - 1)) {
    array_index++;
  } else {
    data_ready = true;
    array_index = 0;
  }
}

// Function to read potentiometer and update sample rate
void updateSampleRate() {
  // Read potentiometer value (0-4095 for 12-bit ADC)
  int potValue = analogRead(POT_PIN);
  
  // Map potentiometer value to sample rate range (1-20ms)
  uint32_t new_rate_ms = map(potValue, 0, 4095, MIN_SAMPLE_RATE_MS, MAX_SAMPLE_RATE_MS);
  uint32_t new_rate_us = new_rate_ms * 1000;
  
  // Only update if changed significantly (avoid jitter)
  if (abs((int)(new_rate_us - sample_rate_us)) > 500) {
    sample_rate_us = new_rate_us;
    analogData.sample_rate_us = sample_rate_us;
    
    // Reconfigure timer
    timerAlarmWrite(timer, sample_rate_us, true);
  }
}

void IRAM_ATTR send_packet() {
  // Send the analog data struct using UDP
  udp.connect(multicastIP, multicastPort);
  udp.write((uint8_t *)&analogData, sizeof(analogData));
  
  // Reset flags
  array_index = 0;
  data_ready = false;
  
  udp.close();
  
  // Increment the packet index
  analogData.index++;
}

// Setup function
void setup() {
  // Initialize analog pins
  analogSetAttenuation(ADC_11db); // Full range: 0-3.3V
  analogReadResolution(12);        // 12-bit resolution (0-4095)
  
  // Initialize the analog data index to zero
  array_index = 0;
  analogData.index = 0;

#ifdef AP_mode_on // if ap mode, start and configure AP
  WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
#endif

#ifndef AP_mode_on // if not AP mode, start STA
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }
#endif

  esp_wifi_set_ps(WIFI_PS_NONE);
  
  // Read initial sample rate from potentiometer
  int potValue = analogRead(POT_PIN);
  uint32_t initial_rate_ms = map(potValue, 0, 4095, MIN_SAMPLE_RATE_MS, MAX_SAMPLE_RATE_MS);
  sample_rate_us = initial_rate_ms * 1000;
  analogData.sample_rate_us = sample_rate_us;
  
  // Initialize hardware timer
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1MHz), count up
  timerAttachInterrupt(timer, &onTimer, true); // Attach interrupt
  timerAlarmWrite(timer, sample_rate_us, true); // Set alarm value, auto-reload
  timerAlarmEnable(timer); // Enable timer alarm
}

// Loop function
void loop() {
  // Check if data is ready to send
  if (data_ready) {
    send_packet();
  }
  
  // Periodically update sample rate from potentiometer
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 100) { // Check every 100ms
    updateSampleRate();
    lastUpdate = millis();
  }
  
  yield();
}
