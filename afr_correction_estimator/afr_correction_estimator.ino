// AFR and Correction Estimator
// This code analyzes lambda probe and injector signals to estimate AFR,
// its deviation, and correlations between signals.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

// Define the analog pins for the lambda probe and the injector signals
#ifdef ARDUINO
const int lambdaPin = A0;
const int injectorPin = A1;
#else
const int lambdaPin = 0;
const int injectorPin = 1;
#endif

// Define the sampling frequency in Hz
#define FREQ 100

// Define the threshold voltage for the lambda probe (0.45 V)
#define THRESHOLD (0.45 / 5.0 * 1023.0)

// Define the stoichiometric AFR for gasoline (14.7)
#define STOICH 14.7

// Define the constants for the AFR calculation (simplified linear model)
#define K1 0.001 // slope
#define C1 0.8 // intercept

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

// Variables for peak detection
int peak_value = 0;
int valley_value = 1023;
unsigned long peak_time = 0;
unsigned long valley_time = 0;

void setup() {
  Serial.begin(115200);
}

void analyze_data() {
  peak_value = 0;
  valley_value = 1023;
  peak_time = 0;
  valley_time = 0;

  int state = 0; // 0: initial, 1: rising, -1: falling
  int prev_value = lambda_buffer[0];

  for (int i = 1; i < BUFFER_SIZE; i++) {
    int curr_value = lambda_buffer[i];
    if (curr_value > prev_value) {
      if (state == -1) {
        valley_value = prev_value;
        valley_time = time_buffer[i - 1];
      }
      state = 1;
    } else if (curr_value < prev_value) {
      if (state == 1) {
        peak_value = prev_value;
        peak_time = time_buffer[i - 1];
      }
      state = -1;
    }
    prev_value = curr_value;
  }
}

void calculate_afr() {
  if (peak_value > THRESHOLD && valley_value < THRESHOLD) {
    int avg_value = (peak_value + valley_value) / 2;
    unsigned long total_time = time_buffer[BUFFER_SIZE - 1] - time_buffer[0];

    float duty_cycle = 0;
    if (total_time > 0) {
        // Simple duty cycle based on peak/valley times (one cycle)
        long cycle_time = abs((long)peak_time - (long)valley_time) * 2;
        if (cycle_time > 0) {
            duty_cycle = (float)abs((long)peak_time - (long)valley_time) / (float)cycle_time;
        }
    }

    // Narrowband approximation
    afr = STOICH / (K1 * avg_value + C1);

    float frequency = 0;
    if (total_time > 0) {
        frequency = 1000.0 / total_time; // This is actually buffer frequency, not signal
    }
    deviation = (afr - STOICH);
  } else {
    afr = STOICH;
    deviation = 0;
  }
}

void calculate_injector_stats() {
  float sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += injector_buffer[i];
  }
  float mean = sum / BUFFER_SIZE;

  float std_dev = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    std_dev += pow(injector_buffer[i] - mean, 2);
  }
  std_dev = sqrt(std_dev / BUFFER_SIZE);

  if (mean > 0) {
    injector_rate = std_dev / mean;
  } else {
    injector_rate = 0;
  }
  injector_magnitude = std_dev;
}

void calculate_lambda_correlation() {
  float mean_l = 0;
  float mean_i = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    mean_l += lambda_buffer[i];
    mean_i += injector_buffer[i];
  }
  mean_l /= BUFFER_SIZE;
  mean_i /= BUFFER_SIZE;

  float numerator = 0;
  float den_l = 0;
  float den_i = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    float dl = lambda_buffer[i] - mean_l;
    float di = injector_buffer[i] - mean_i;
    numerator += dl * di;
    den_l += dl * dl;
    den_i += di * di;
  }

  float denominator = sqrt(den_l * den_i);
  if (denominator > 0) {
    lambda_correlation = numerator / denominator;
  } else {
    lambda_correlation = 0;
  }
}

void print_results() {
  Serial.print("AFR: "); Serial.print(afr);
  Serial.print(" | Dev: "); Serial.print(deviation);
  Serial.print(" | InjRate: "); Serial.print(injector_rate);
  Serial.print(" | Corr: "); Serial.println(lambda_correlation);
}

void loop() {
  lambda_buffer[buffer_index] = analogRead(lambdaPin);
  injector_buffer[buffer_index] = analogRead(injectorPin);
  time_buffer[buffer_index] = millis();

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

  delay(1000 / FREQ);
}
