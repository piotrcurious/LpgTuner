// Define analog pins for lambda probe and injector signals
const int lambdaPin = A0;
const int injectorPin = A1;

// Define voltage range for lambda probe signal
const float lambdaMin = 0.1;
const float lambdaMax = 0.9;

// Define voltage range for injector signal
const float injectorMin = 0;
const float injectorMax = 5;

// Define ideal stoichiometric ratio for gasoline engines
const float stoichRatio = 14.7;

// Define threshold value for AFR crossing stoichiometric point
const float threshold = 0.05;

// Define filter window size for moving average
const int windowSize = 10;

// Define variables for storing sensor readings
float lambdaVoltage = 0;
float injectorVoltage = 0;
float AFR = 0;

// Define variables for storing filtered values
float lambdaFiltered = 0;
float injectorFiltered = 0;
float AFRFiltered = 0;

// Define arrays for storing window values
float lambdaWindow[windowSize];
float injectorWindow[windowSize];
float AFRWindow[windowSize];

// Define variables for storing previous values
float lambdaPrev = 0;
float injectorPrev = 0;
float AFRPrev = 0;

// Define variables for storing oscillation parameters
float period = 0;
float dutyCycle = 0;
float frequency = 0;
float amplitude = 0;

// Define variables for storing timer values
unsigned long startTime = 0;
unsigned long endTime = 0;
unsigned long highTime = 0;
unsigned long lowTime = 0;

// Define variables for storing state values
bool highState = false;
bool lowState = false;
bool risingEdge = false;
bool fallingEdge = false;

void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);

  // Initialize window arrays with zero values
  for (int i = 0; i < windowSize; i++) {
    lambdaWindow[i] = 0;
    injectorWindow[i] = 0;
    AFRWindow[i] = 0;
  }
}

void loop() {
  // Read analog values from lambda probe and injector pins
  lambdaVoltage = analogRead(lambdaPin) * (5.0 / 1023.0);
  injectorVoltage = analogRead(injectorPin) * (5.0 / 1023.0);

  // Calculate AFR from lambda probe voltage using linear interpolation
  AFR = map(lambdaVoltage, lambdaMin, lambdaMax, stoichRatio * (1 + threshold), stoichRatio * (1 - threshold));

  // Apply moving average filter to smooth out noise in sensor readings
  lambdaFiltered = movingAverage(lambdaVoltage, lambdaWindow);
  injectorFiltered = movingAverage(injectorVoltage, injectorWindow);
  AFRFiltered = movingAverage(AFR, AFRWindow);

  // Detect rising and falling edges of AFR crossing stoichiometric point
  risingEdge = (AFRPrev < stoichRatio) && (AFRFiltered >= stoichRatio);
  fallingEdge = (AFRPrev > stoichRatio) && (AFRFiltered <= stoichRatio);

  // Measure period and duty cycle of AFR oscillation using timer
  if (risingEdge) {
    // Start timer at rising edge
    startTime = millis();
    // Reset low state flag
    lowState = false;
    // Set high state flag
    highState = true;
  }
  else if (fallingEdge) {
    // Stop timer at falling edge
    endTime = millis();
    // Calculate period as difference between start and end times
    period = (endTime - startTime) / 1000.0;
    // Calculate high time as difference between end and low times
    highTime = (endTime - lowTime) / 1000.0;
    // Calculate duty cycle as ratio of high time to period
    dutyCycle = highTime / period;
    // Reset high state flag
    highState = false;
    // Set low state flag
    lowState = true;
  }
  else if (highState) {
    // Update low time at high state
    lowTime = millis();
  }

  // Calculate frequency and amplitude of AFR oscillation from period and duty cycle
  frequency = 1 / period;
  amplitude = stoichRatio * threshold * 2 * dutyCycle;

  // Display AFR, frequency, and amplitude values on serial monitor
  Serial.print("AFR: ");
  Serial.print(AFRFiltered, 2);
  Serial.print(" | Frequency: ");
  Serial.print(frequency, 2);
  Serial.print(" Hz | Amplitude: ");
  Serial.print(amplitude, 2);
  Serial.println();

  // Plot AFR, frequency, and amplitude values on serial plotter
  Serial.print(AFRFiltered);
  Serial.print(",");
  Serial.print(frequency);
  Serial.print(",");
  Serial.println(amplitude);

  // Update previous values with current values
  lambdaPrev = lambdaFiltered;
  injectorPrev = injectorFiltered;
  AFRPrev = AFRFiltered;
}

// Function to calculate moving average of a sensor reading using a window array
float movingAverage(float reading, float window[]) {
  // Shift window values to the left by one position
  for (int i = 0; i < windowSize - 1; i++) {
    window[i] = window[i + 1];
  }
  // Add new reading to the rightmost position of the window
  window[windowSize - 1] = reading;
  
  // Calculate the sum of the window values
  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += window[i];
  }
  
  // Return the average of the window values
  return sum / windowSize;
}
