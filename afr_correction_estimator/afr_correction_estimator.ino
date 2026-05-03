// AFR and Correction Estimator
// This code analyzes lambda probe and injector signals to estimate AFR,
// its deviation, and correlations between signals.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

// Define the analog pins for the lambda probe and the injector signals
#if defined(ARDUINO) && !defined(UNIT_TEST)
#define LP_PIN A0
#define IJ_PIN A1
#else
#define LP_PIN 0
#define IJ_PIN 1
#endif

#ifdef UNIT_TEST
extern int analogRead(int pin);
#define ANALOG_READ_MOCK(p) analogRead(p)
#else
#define ANALOG_READ_MOCK(p) analogRead(p)
#endif

// Define the sampling frequency in Hz
#define FREQ 100

// Define the stoichiometric AFR for gasoline (14.7)
#define STOICH 14.7

// Define a buffer size for storing samples
#define BUFFER_SIZE 100

// Declare global variables for storing samples and timestamps
int lambda_buffer[BUFFER_SIZE];
int injector_buffer[BUFFER_SIZE];
unsigned long time_buffer[BUFFER_SIZE];

// Declare global variables for keeping track of buffer index and overflow flag
int buffer_index = 0;
bool buffer_full = false;

// Declare global variables for storing results
float afr = 0;
float deviation = 0;
float injector_rate = 0;
float injector_magnitude = 0;
float lambda_correlation = 0;
int best_lag = 0;

unsigned long last_sample_time = 0xFFFFFFFF;
const unsigned long sample_interval = 1000 / FREQ;

// Threshold for narrowband O2 sensor (0.45V)
const int THRESHOLD = (int)(0.45 / 5.0 * 1023.0);

// Variables for peak detection
float avg_peak = 0;
float avg_valley = 0;
int peak_count = 0;
int valley_count = 0;

void setup() {
  Serial.begin(115200);
  last_sample_time = 0xFFFFFFFF;
  buffer_index = 0;
  buffer_full = false;
}

void analyze_data() {
  float sum_peaks = 0;
  float sum_valleys = 0;
  peak_count = 0;
  valley_count = 0;

  int state = 0; // 0: initial, 1: rising, -1: falling
  int prev_value = lambda_buffer[0];

  for (int i = 1; i < BUFFER_SIZE; i++) {
    int curr_value = lambda_buffer[i];
    if (curr_value > prev_value) {
      if (state == -1) {
        sum_valleys += prev_value;
        valley_count++;
      }
      state = 1;
    } else if (curr_value < prev_value) {
      if (state == 1) {
        sum_peaks += prev_value;
        peak_count++;
      }
      state = -1;
    }
    prev_value = curr_value;
  }

  if (peak_count > 0) avg_peak = sum_peaks / peak_count;
  else avg_peak = 0;

  if (valley_count > 0) avg_valley = sum_valleys / valley_count;
  else avg_valley = 1023;
}

void calculate_afr() {
  float estimated_lambda = 1.0;

  if (peak_count > 0 && valley_count > 0 && avg_peak > THRESHOLD && avg_valley < THRESHOLD) {
    // Narrowband O2 sensors oscillate around stoichiometry.
    float mean_lambda_val = (avg_peak + avg_valley) / 2.0;
    float voltage = (mean_lambda_val / 1023.0) * 5.0;
    estimated_lambda = 1.0 - (voltage - 0.45) * 0.2;
  } else {
    // No clear oscillation, use raw average
    double sum = 0;
    for(int i = 0; i < BUFFER_SIZE; i++) sum += lambda_buffer[i];
    float avg_raw = (float)(sum / BUFFER_SIZE);
    float voltage = (avg_raw / 1023.0) * 5.0;
    estimated_lambda = 1.0 - (voltage - 0.45) * 0.2;
  }

  // Clamp lambda to realistic narrowband range [0.8, 1.2]
  if (estimated_lambda < 0.8) estimated_lambda = 0.8;
  if (estimated_lambda > 1.2) estimated_lambda = 1.2;

  afr = estimated_lambda * STOICH;
  deviation = (afr - STOICH);
}

void calculate_injector_stats() {
  double sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += injector_buffer[i];
  }
  double mean = sum / BUFFER_SIZE;

  double sum_sq_diff = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum_sq_diff += pow((double)injector_buffer[i] - mean, 2);
  }
  double std_dev = sqrt(sum_sq_diff / BUFFER_SIZE);

  if (mean > 0) {
    injector_rate = (float)(std_dev / mean);
  } else {
    injector_rate = 0;
  }
  injector_magnitude = (float)std_dev;
}

void calculate_lambda_correlation() {
  const int MAX_LAG = 20;
  double max_corr = 0;
  int lag_at_max = 0;

  for (int lag = 0; lag <= MAX_LAG; lag++) {
    double current_mean_l = 0;
    double current_mean_i = 0;
    int count = 0;
    for (int i = lag; i < BUFFER_SIZE; i++) {
      current_mean_l += (double)lambda_buffer[i];
      current_mean_i += (double)injector_buffer[i - lag];
      count++;
    }
    if (count < 2) continue;
    current_mean_l /= count;
    current_mean_i /= count;

    double numerator = 0;
    double sum_sq_l = 0;
    double sum_sq_i = 0;

    for (int i = lag; i < BUFFER_SIZE; i++) {
      double dl = (double)lambda_buffer[i] - current_mean_l;
      double di = (double)injector_buffer[i - lag] - current_mean_i;
      numerator += dl * di;
      sum_sq_l += dl * dl;
      sum_sq_i += di * di;
    }

    double denominator = sqrt(sum_sq_l * sum_sq_i);
    double current_corr = (denominator > 0) ? (numerator / denominator) : 0;

    if (fabs(current_corr) > fabs(max_corr) + 0.00001) {
      max_corr = current_corr;
      lag_at_max = lag;
    }
  }

  lambda_correlation = (float)max_corr;
  best_lag = lag_at_max;
}

void print_results() {
  Serial.print("AFR: "); Serial.print(afr);
  Serial.print(" | Dev: "); Serial.print(deviation);
  Serial.print(" | InjRate: "); Serial.print(injector_rate);
  Serial.print(" | Corr: "); Serial.print(lambda_correlation);
  Serial.print(" | Lag: "); Serial.println(best_lag);
}

void loop() {
  unsigned long now = millis();
  bool first_run = (last_sample_time == 0xFFFFFFFF);

  if (first_run || (now - last_sample_time >= sample_interval)) {
    if (first_run) last_sample_time = now;
    else last_sample_time += sample_interval;

    lambda_buffer[buffer_index] = ANALOG_READ_MOCK(LP_PIN);
    injector_buffer[buffer_index] = ANALOG_READ_MOCK(IJ_PIN);
    time_buffer[buffer_index] = now;

    buffer_index++;
    if (buffer_index >= BUFFER_SIZE) {
      buffer_index = 0;
      buffer_full = true;
    }

    if (buffer_full) {
      analyze_data();
      calculate_afr();
      calculate_injector_stats();
      calculate_lambda_correlation();
      print_results();
      buffer_full = false;
    }
  }
}
