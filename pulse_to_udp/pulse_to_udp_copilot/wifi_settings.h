//WIFI and UDP multicast includes


//ESP8266 
//#include <ESP8266WiFi.h>
//#include <ESPAsyncUDP.h>

//ESP32
#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

const char* ssid = "voltage";
const char* password = "irrolling12";
const uint8_t channel = 1; 
const bool hidden = 0;
const uint8_t max_connection = 8; 
const uint16_t beacon_interval = 500; 
const uint16_t timeout_period = 5000; // expected delay inbetween packets (for accounting display) 
#define AP_mode_on 1 ; // set to 1 to compile AP node

uint16_t multicastPort = 5683;  // local port to listen on
IPAddress multicastIP(224,0,1,187);
