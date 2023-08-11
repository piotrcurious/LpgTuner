// Define the analog pins for the lambda probe and the injector signals
#define LAMBDA_PIN A0
#define INJECTOR_PIN A1

// Define the sampling frequency in Hz
#define FREQ 100

// Define the threshold voltage for the lambda probe (0.45 V)
#define THRESHOLD 0.45 / 5 * 1023

// Define the stoichiometric AFR for gasoline (14.7)
#define STOICH 14.7

// Define the constants for the AFR calculation
#define K1 0.01 // slope of the rich side of the lambda curve
#define K2 0.03 // slope of the lean side of the lambda curve
#define C1 0.8 // intercept of the rich side of the lambda curve
#define C2 0.1 // intercept of the lean side of the lambda curve

// Define a buffer size for storing samples
#define BUFFER_SIZE 100

// Declare global variables for storing samples and timestamps
int lambda_buffer[BUFFER_SIZE];
int injector_buffer[BUFFER_SIZE];
unsigned long time_buffer[BUFFER_SIZE];

// Declare global variables for keeping track of buffer index and overflow flag
int buffer_index = 0;
bool buffer_overflow = false;

// Declare global variables for storing the AFR and deviation values
float afr = 0;
float deviation = 0;

// Declare global variables for storing the injector duty cycle fluctuations rate and magnitude
float injector_rate = 0;
float injector_magnitude = 0;

// Declare global variables for storing the correlation between lambda duty cycle and magnitude
float lambda_correlation = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set the analog pins as inputs
  pinMode(LAMBDA_PIN, INPUT);
  pinMode(INJECTOR_PIN, INPUT);
}

void loop() {
  // Read the analog values from the lambda probe and the injector signals
  int lambda_value = analogRead(LAMBDA_PIN);
  int injector_value = analogRead(INJECTOR_PIN);

  // Get the current timestamp in milliseconds
  unsigned long time_value = millis();

  // Store the values and the timestamp in the buffers
  lambda_buffer[buffer_index] = lambda_value;
  injector_buffer[buffer_index] = injector_value;
  time_buffer[buffer_index] = time_value;

  // Increment the buffer index and wrap around if necessary
  buffer_index++;
  if (buffer_index == BUFFER_SIZE) {
    buffer_index = 0;
    buffer_overflow = true;
  }

  // If the buffer is full, analyze the data and calculate the AFR, deviation, injector rate, injector magnitude, and lambda correlation
  if (buffer_overflow) {
    analyze_data();
    calculate_afr();
    calculate_deviation();
    calculate_injector_rate();
    calculate_injector_magnitude();
    calculate_lambda_correlation();
    print_results();
    buffer_overflow = false;
  }

  // Wait for the next sampling interval
  delay(1000 / FREQ);
}

// This function analyzes the data in the lambda buffer and finds the peak and valley voltages
// and their corresponding timestamps, using a simple peak detection algorithm
void analyze_data() {

  // Declare local variables for storing the peak and valley values and timestamps
  int peak_value = 0;
  int valley_value = 1023;
  unsigned long peak_time = 0;
  unsigned long valley_time = 0;

  // Declare local variables for keeping track of the current state and previous value
  int state = 0; // 0: initial, 1: rising, -1: falling
  int prev_value = lambda_buffer[0];

  // Loop through the lambda buffer and find the peak and valley points
  for (int i = 1; i < BUFFER_SIZE; i++) {
    int curr_value = lambda_buffer[i];
    if (curr_value > prev_value) { // rising edge
      if (state == -1) { // falling to rising transition, valley point found
        valley_value = prev_value;
        valley_time = time_buffer[i - 1];
      }
      state = 1; // update state to rising
    } else if (curr_value < prev_value) { // falling edge
      if (state == 1) { // rising to falling transition, peak point found
        peak_value = prev_value;
        peak_time = time_buffer[i - 1];
      }
      state = -1; // update state to falling
    }
    prev_value = curr_value; // update previous value
  }
}

  // Check if the peak and valley values are valid (above and below the threshold voltage)
  if (peak_value > THRESHOLD && valley_value < THRESHOLD) {
    // Calculate the average voltage and time of the peak and valley points
    int avg_value = (peak_value + valley_value) / 2;
    unsigned long avg_time = (peak_time + valley_time) / 2;

    // Calculate the duty cycle and frequency of the lambda signal
    float duty_cycle = (float)(peak_time - valley_time) / (time_buffer[BUFFER_SIZE - 1] - time_buffer[0]);
    float frequency = 1.0 / ((float)(time_buffer[BUFFER_SIZE - 1] - time_buffer[0]) / 1000);

    // Calculate the AFR from the average voltage using the lambda curve equation
    if (avg_value > THRESHOLD) { // rich side
      afr = STOICH / (K1 * avg_value + C1);
    } else { // lean side
      afr = STOICH / (K2 * avg_value + C2);
    }

    // Calculate the deviation from the stoichiometric point using the duty cycle and frequency
    deviation = duty_cycle * frequency * (STOICH - afr);
  } else {
    // Invalid peak and valley values, set AFR and deviation to zero
    afr = 0;
    deviation = 0;
  }
}

// This function calculates the injector duty cycle fluctuations rate and magnitude
// using the standard deviation and mean of the injector signal
void calculate_injector_rate() {
  // Declare local variables for storing the sum, mean, and standard deviation of the injector signal
  float sum = 0;
  float mean = 0;
  float std_dev = 0;

  // Loop through the injector buffer and calculate the sum and mean of the signal
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += injector_buffer[i];
  }
  mean = sum / BUFFER_SIZE;

  // Loop through the injector buffer again and calculate the standard deviation of the signal
  for (int i = 0; i < BUFFER_SIZE; i++) {
    std_dev += sq(injector_buffer[i] - mean);
  }
  std_dev = sqrt(std_dev / BUFFER_SIZE);

  // Calculate the injector duty cycle fluctuations rate and magnitude using the formula:
  // rate = std_dev / mean
  // magnitude = std_dev * FREQ
  injector_rate = std_dev / mean;
  injector_magnitude = std_dev * FREQ;
}

// This function calculates the correlation between lambda duty cycle and magnitude
// using the Pearson correlation coefficient
void calculate_lambda_correlation() {
  // Declare local variables for storing the lambda duty cycle and magnitude arrays
  float lambda_duty_cycle[BUFFER_SIZE];
  float lambda_magnitude[BUFFER_SIZE];

  // Declare local variables for storing the sums, means, and standard deviations of the lambda duty cycle and magnitude arrays
  float sum_duty_cycle = 0;
  float sum_magnitude = 0;
  float mean_duty_cycle = 0;
  float mean_magnitude = 0;
  float std_dev_duty_cycle = 0;
  float std_dev_magnitude = 0;

  // Declare local variables for storing the numerator and denominator of the correlation coefficient
  float numerator = 0;
  float denominator = 0;

  // Loop through the lambda buffer and calculate the lambda duty cycle and magnitude arrays
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Calculate the lambda duty cycle using the formula:
    // duty_cycle = (lambda_value - THRESHOLD) / (1023 - THRESHOLD)
    // where lambda_value is the voltage of the lambda signal in ADC counts
    lambda_duty_cycle[i] = (float)(lambda_buffer[i] - THRESHOLD) / (1023 - THRESHOLD);

    // Calculate the lambda magnitude using the formula:
    // magnitude = abs(lambda_value - THRESHOLD)
    lambda_magnitude[i] = abs(lambda_buffer[i] - THRESHOLD);

    // Calculate the sums of the lambda duty cycle and magnitude arrays
    sum_duty_cycle += lambda_duty_cycle[i];
    sum_magnitude += lambda_magnitude[i];
  }

  // Calculate the means of the lambda duty cycle and magnitude arrays
  mean_duty_cycle = sum_duty_cycle / BUFFER_SIZE;
  mean_magnitude = sum_magnitude / BUFFER_SIZE;

  // Loop through the lambda duty cycle and magnitude arrays again and calculate the standard deviations and the numerator of the correlation coefficient
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Calculate the standard deviations of the lambda duty cycle and magnitude arrays
    std_dev_duty_cycle += sq(lambda_duty_cycle[i] - mean_duty_cycle);
    std_dev_magnitude += sq(lambda_magnitude[i] - mean_magnitude);

    // Calculate the numerator of the correlation coefficient using the formula:
    // numerator = sum((lambda_duty_cycle[i] - mean_duty_cycle) * (lambda_magnitude[i] - mean_magnitude))
    numerator += (lambda_duty_cycle[i] - mean_duty_cycle) * (lambda_magnitude[i] - mean_magnitude);
  }
  std_dev_duty_cycle = sqrt(std_dev_duty_cycle / BUFFER_SIZE);
  std_dev_magnitude = sqrt(std_dev_magnitude / BUFFER_SIZE);

  // Calculate the denominator of the correlation coefficient using the formula:
  // denominator = std_dev_duty_cycle * std_dev_magnitude
  denominator = std_dev_duty_cycle * std_dev_magnitude;

  // Calculate the correlation between lambda duty cycle and magnitude using the formula:
  // correlation = numerator / denominator
  lambda_correlation = numerator / denominator;
}

// This function prints the results to the serial monitor
void print_results() {
  // Print the AFR, deviation, injector rate, injector magnitude, and lambda correlation values to the serial monitor
  Serial.print("AFR: ");
  Serial.println(afr);
  Serial.print("Deviation: ");
  Serial.println(deviation);
  Serial.print("Injector rate: ");
  Serial.println(injector_rate);
  Serial.print("Injector magnitude: ");
  Serial.println(injector_magnitude);
  Serial.print("Lambda correlation: ");
  Serial.println(lambda_correlation);
}
