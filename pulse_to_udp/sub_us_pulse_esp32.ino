// Dreamed by copilot.

// This code is for demonstration purposes only and has not been tested or verified.

// Define the pin to measure the pulse length
#define PULSE_PIN 2

// Define the timer number to use (0, 1 or 3)
#define TIMER_NUM 0

// Define the timer frequency in Hz (max 80 MHz)
#define TIMER_FREQ 80000000

// Define the timer width in bits (1 to 64)
#define TIMER_WIDTH 64

// Define a variable to store the pulse start time
uint64_t pulse_start = 0;

// Define a variable to store the pulse length
uint64_t pulse_length = 0;

// Define a flag to indicate a new pulse measurement
bool pulse_ready = false;

// Define an interrupt service routine for the pulse pin
void IRAM_ATTR pulse_isr() {
  // Get the current timer count
  uint64_t timer_count = timerRead(TIMER_NUM);

  // Check the state of the pulse pin
  if (digitalRead(PULSE_PIN) == HIGH) {
    // Pulse rising edge, store the start time
    pulse_start = timer_count;
  } else {
    // Pulse falling edge, calculate the length and set the flag
    pulse_length = timer_count - pulse_start;
    pulse_ready = true;
  }
}

// Define a function to initialize the timer
void init_timer() {
  // Set the timer configuration
  timer_config_t config;
  config.divider = (TIMER_BASE_CLK / TIMER_FREQ);
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_PAUSE;
  config.alarm_en = TIMER_ALARM_DIS;
  config.intr_type = TIMER_INTR_LEVEL;
  config.auto_reload = TIMER_AUTORELOAD_DIS;

  // Initialize the timer with the configuration
  timer_init(TIMER_GROUP_0, (timer_idx_t)TIMER_NUM, &config);

  // Reset the timer counter
  timerSetCounterValue(TIMER_NUM, 0);

  // Start the timer
  timerStart(TIMER_NUM);
}

// Define a function to initialize the pulse pin
void init_pulse() {
  // Set the pulse pin as input with pull-up
  pinMode(PULSE_PIN, INPUT_PULLUP);

  // Attach an interrupt to the pulse pin on both edges
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), pulse_isr, CHANGE);
}

// Define a function to print the pulse length in microseconds
void print_pulse() {
  // Check if a new pulse measurement is ready
  if (pulse_ready) {
    // Convert the pulse length to microseconds
    double pulse_us = (double)pulse_length / (double)TIMER_FREQ * 1000000.0;

    // Print the pulse length
    Serial.print("Pulse length: ");
    Serial.print(pulse_us, 6);
    Serial.println(" us");

    // Reset the flag
    pulse_ready = false;
  }
}

void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);

  // Initialize the timer
  init_timer();

  // Initialize the pulse pin
  init_pulse();
}

void loop() {
  // Print the pulse length if available
  print_pulse();
}
