#ifndef MOCK_ESP32_H
#define MOCK_ESP32_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdarg.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

// Arduino Mocks
#define IRAM_ATTR
#define RISING 1
#define FALLING 2
#define INPUT_PULLUP 3

struct String : public std::string {
    String() : std::string() {}
    String(const char* s) : std::string(s) {}
    String(const std::string& s) : std::string(s) {}
    String substring(size_t from, size_t to = -1) const {
        if (to == (size_t)-1) return String(this->substr(from));
        return String(this->substr(from, to - from));
    }
    int indexOf(char c) const {
        size_t pos = this->find(c);
        if (pos == std::string::npos) return -1;
        return (int)pos;
    }
    const char* c_str() const { return std::string::c_str(); }
    size_t length() const { return std::string::length(); }
};

void delay(int ms) {}
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
void pinMode(int pin, int mode) {}
void attachInterrupt(int pin, void (*fn)(), int mode) {}
void detachInterrupt(int pin) {}
int digitalPinToInterrupt(int pin) { return pin; }

struct SerialMock {
    void begin(int baud) {}
    void print(const String& s) {}
    void print(const char* s) {}
    void println(const String& s) {}
    void println(const char* s) {}
    void printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        va_end(args);
    }
};
extern SerialMock Serial;

struct ESPMock {
    void restart() {}
    uint32_t getFreeHeap() { return 100000; }
};
extern ESPMock ESP;

// WiFi Mocks
#define WL_CONNECTED 3
struct WiFiMock {
    void begin(const char* s, const char* p) {}
    int status() { return WL_CONNECTED; }
    String localIP() { return String("192.168.1.1"); }
    void softAP(const char* s, const char* p) {}
    String softAPIP() { return String("192.168.4.1"); }
};
extern WiFiMock WiFi;

// AsyncWebServer Mocks
struct AsyncWebServerRequest {
    void send(int code, const char* type, const char* body) {}
};
enum AwsEventType { WS_EVT_CONNECT, WS_EVT_DISCONNECT, WS_EVT_DATA };
struct AsyncWebSocketClient {
    uint32_t id() { return 1; }
};
struct AwsFrameInfo {
    bool final;
    uint32_t index;
    uint32_t len;
};
struct AsyncWebSocket {
    AsyncWebSocket(const char* url) {}
    void onEvent(void (*fn)(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType, void*, uint8_t*, size_t)) {}
    void binaryAll(uint8_t* data, size_t len) {}
    int count() { return 1; }
    void cleanupClients() {}
};
struct AsyncWebServer {
    AsyncWebServer(int port) {}
    void addHandler(AsyncWebSocket* ws) {}
    void on(const char* url, int method, void (*fn)(AsyncWebServerRequest*)) {}
    void begin() {}
};
#define HTTP_GET 1

// Preferences Mock
struct Preferences {
    void begin(const char* name, bool readonly) {}
    void putString(const char* key, String val) {}
    void putInt(const char* key, int val) {}
    String getString(const char* key, const char* def) { return String(def); }
    int getInt(const char* key, int def) { return def; }
    void end() {}
};

// ESP-IDF ADC Continuous Mocks
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERROR_CHECK(x) (x)
typedef void* adc_continuous_handle_t;
typedef int adc_channel_t;
#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_3 3
#define ADC_CHANNEL_6 6
#define ADC_CHANNEL_7 7
#define ADC_UNIT_1 1
#define ADC_ATTEN_DB_11 3
#define ADC_WIDTH_BIT_12 12
#define ADC_BITWIDTH_12 12
#define ADC_CONV_SINGLE_UNIT_1 1
#define ADC_DIGI_OUTPUT_FORMAT_TYPE1 1
#define ADC_READ_TIMEOUT_MS 10

struct adc_continuous_handle_cfg_t {
    uint32_t max_store_buf_size;
    uint32_t conv_frame_size;
};
struct adc_digi_pattern_config_t {
    uint8_t atten;
    uint8_t channel;
    uint8_t unit;
    uint8_t bit_width;
};
struct adc_continuous_config_t {
    uint32_t pattern_num;
    adc_digi_pattern_config_t* adc_pattern;
    uint32_t sample_freq_hz;
    uint8_t conv_mode;
    uint8_t format;
};
struct adc_digi_output_data_t {
    struct {
        uint16_t data : 12;
        uint16_t channel : 4;
    } type1;
};

inline esp_err_t adc_continuous_new_handle(adc_continuous_handle_cfg_t* cfg, adc_continuous_handle_t* handle) { return ESP_OK; }
inline esp_err_t adc_continuous_config(adc_continuous_handle_t handle, adc_continuous_config_t* cfg) { return ESP_OK; }
inline esp_err_t adc_continuous_start(adc_continuous_handle_t handle) { return ESP_OK; }
inline esp_err_t adc_continuous_read(adc_continuous_handle_t handle, uint8_t* buf, uint32_t len, uint32_t* out_len, uint32_t timeout) { return ESP_OK; }

typedef struct {
    uint32_t AC;
} esp_adc_cal_characteristics_t;
inline void esp_adc_cal_characterize(int unit, int atten, int width, int vref, esp_adc_cal_characteristics_t* chars) {}

// FreeRTOS Mocks
typedef void* TaskHandle_t;
inline void xTaskCreatePinnedToCore(void (*fn)(void*), const char* name, int stack, void* arg, int prio, TaskHandle_t* handle, int core) {}
inline void vTaskDelay(int ticks) {}
#define pdMS_TO_TICKS(ms) (ms)
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE* mux) {}
inline void portEXIT_CRITICAL(portMUX_TYPE* mux) {}
inline void portENTER_CRITICAL_ISR(portMUX_TYPE* mux) {}
inline void portEXIT_CRITICAL_ISR(portMUX_TYPE* mux) {}

#define constrain(val, min, max) ((val)<(min)?(min):((val)>(max)?(max):(val)))
inline int random(int max) { return 0; }

#endif
