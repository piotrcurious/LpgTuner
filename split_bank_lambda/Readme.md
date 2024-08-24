Assume the system is car engine with four cylinders grouped in two subgroups of two cylinders, each group sharing exhaust manifold with own narrowband lambda probe. For better accuracy read each lambda probe as analog input. Create esp32 Arduino code for a injection pulse length offset controller, assuming that injection pulse length is sent to esp32 by analog input from injection computer and esp32 controls injection length offset for each subgroup by using DACs. DAC should only output offset value with 128 as 0. Design the controller so it tries to maintain slightly lean reading in one subgroup and slightly rich in the other cylinder subgroup and emulates narrowband lambda probe output if both offsets are not symmetrical or exceed the DAC constraints. 

Creating a control system for an ESP32 that adjusts the fuel injection pulse length based on the readings from two narrowband lambda sensors is a complex task. Below is an example of how such a system could be implemented in Arduino code. The ESP32 reads the lambda probes' values, adjusts the injection pulse length using DAC outputs, and emulates a narrowband lambda probe output if the adjustments exceed certain constraints.

### Assumptions:
- **Lambda Probes:** Narrowband lambda sensors for each subgroup (two sensors).
- **Injection Pulse Length:** Analog input from the engine's injection computer.
- **DAC Output:** Each DAC controls the injection pulse length offset for each subgroup.
- **Offset Value:** `128` is the baseline (no offset), values above 128 increase the pulse length, and values below 128 decrease the pulse length.
- **Target AFR (Air-Fuel Ratio):** One subgroup should be slightly lean, and the other slightly rich.

### ESP32 Pin Configuration:
- `A0`: Analog input from the injection computer (original pulse length).
- `A1`: Analog input from the first lambda sensor.
- `A2`: Analog input from the second lambda sensor.
- `DAC1 (GPIO 25)`: Offset output for the first subgroup.
- `DAC2 (GPIO 26)`: Offset output for the second subgroup.

### Example Arduino Code:

```cpp
// Pin definitions
#define INJECTION_PULSE_PIN  34  // Analog input from the injection computer
#define LAMBDA1_PIN          35  // Analog input from lambda sensor 1
#define LAMBDA2_PIN          36  // Analog input from lambda sensor 2
#define DAC1_PIN             25  // DAC output for subgroup 1
#define DAC2_PIN             26  // DAC output for subgroup 2

// Desired target voltages for lean and rich conditions
#define LEAN_TARGET          0.45  // V for slightly lean condition
#define RICH_TARGET          0.65  // V for slightly rich condition

// DAC output constraints
#define DAC_MIN              0
#define DAC_MAX              255
#define DAC_MID              128

// Proportional gain for the controller
#define KP                   2.0

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Setup analog inputs and DAC outputs
  analogReadResolution(12);  // ESP32 default ADC resolution is 12-bit (0-4095)
  analogSetAttenuation(ADC_11db);  // Full-scale voltage range (0-3.3V)
}

void loop() {
  // Read the current injection pulse length from the injection computer
  int injectionPulseRaw = analogRead(INJECTION_PULSE_PIN);
  float injectionPulseVoltage = injectionPulseRaw * (3.3 / 4095.0);

  // Read lambda sensors
  int lambda1Raw = analogRead(LAMBDA1_PIN);
  int lambda2Raw = analogRead(LAMBDA2_PIN);
  float lambda1Voltage = lambda1Raw * (3.3 / 4095.0);
  float lambda2Voltage = lambda2Raw * (3.3 / 4095.0);

  // Calculate the error for each subgroup
  float error1 = LEAN_TARGET - lambda1Voltage;  // Subgroup 1 should be lean
  float error2 = RICH_TARGET - lambda2Voltage;  // Subgroup 2 should be rich

  // Calculate the offset
  int offset1 = DAC_MID + (int)(KP * error1 * 128.0);
  int offset2 = DAC_MID + (int)(KP * error2 * 128.0);

  // Constrain the offset values to DAC limits
  offset1 = constrain(offset1, DAC_MIN, DAC_MAX);
  offset2 = constrain(offset2, DAC_MIN, DAC_MAX);

  // Output the offset values via DACs
  dacWrite(DAC1_PIN, offset1);
  dacWrite(DAC2_PIN, offset2);

  // Check if the offsets are not symmetrical or exceed DAC limits
  if (abs(offset1 - offset2) > 10 || offset1 == DAC_MIN || offset1 == DAC_MAX || offset2 == DAC_MIN || offset2 == DAC_MAX) {
    // Emulate narrowband lambda probe output
    // (You can define specific logic here, such as setting DAC outputs to a fixed voltage)
    dacWrite(DAC1_PIN, DAC_MID);
    dacWrite(DAC2_PIN, DAC_MID);
    Serial.println("Emulating lambda probe output due to non-symmetry or limit exceedance.");
  }

  // Debugging output
  Serial.print("Lambda1: "); Serial.print(lambda1Voltage); Serial.print("V, Offset1: "); Serial.println(offset1);
  Serial.print("Lambda2: "); Serial.print(lambda2Voltage); Serial.print("V, Offset2: "); Serial.println(offset2);

  // Delay to allow time for the engine's conditions to stabilize
  delay(100);
}
```

### Code Explanation:
1. **Lambda Reading and Processing:** 
   - Reads the lambda sensor values (simulated as voltage).
   - Computes the error relative to the target (lean or rich).

2. **Offset Calculation:**
   - Uses proportional control (P-controller) to calculate the necessary offset for each subgroup.

3. **Output to DAC:**
   - Writes the calculated offsets to the DACs, which modify the injection pulse length.

4. **Error Handling:**
   - If the offsets for the two subgroups become too different or hit DAC limits, the code switches to emulating a narrowband lambda output (safety mode).

5. **Debugging:**
   - The serial monitor output helps in debugging the system by displaying real-time sensor readings and calculated offsets.

### Notes:
- **Proportional Gain (KP):** Adjust this parameter based on the engine's response. Too high a value might cause instability, while too low might lead to sluggish response.
- **Lambda Target Values:** Fine-tune these values to achieve the desired AFR in each subgroup.
- **Error Handling:** The code can be expanded to better handle edge cases or to implement a more sophisticated control strategy.

This code provides a framework that should be tested and tuned on actual hardware with the specific engine setup to ensure proper operation.



improve the code to make it emulate narrowband lambda probe by setting two digital output pin depending on lean or rich condition, encoding one pin as lean/rich and second pin being enabled outputting lean/rich signal if the error magnitude is larger than threshold . Make the pin inputs when error is within the offset. 

To implement the improvements, we'll add two digital output pins for each lambda probe to emulate the narrowband lambda output. Here's how it works:

- **Lambda Condition Pin:** Indicates whether the condition is lean (LOW) or rich (HIGH).
- **Enable Pin:** Activates the output signal (set to OUTPUT mode) if the error exceeds a certain threshold, and deactivates it (set to INPUT mode) otherwise.

### Revised Arduino Code:

```cpp
// Pin definitions
#define INJECTION_PULSE_PIN  34  // Analog input from the injection computer
#define LAMBDA1_PIN          35  // Analog input from lambda sensor 1
#define LAMBDA2_PIN          36  // Analog input from lambda sensor 2
#define DAC1_PIN             25  // DAC output for subgroup 1
#define DAC2_PIN             26  // DAC output for subgroup 2

// Digital output pins for narrowband lambda probe emulation
#define LAMBDA1_CONDITION_PIN  27  // Indicates lean (LOW) or rich (HIGH) for subgroup 1
#define LAMBDA1_ENABLE_PIN     14  // Enable signal for lambda 1 condition
#define LAMBDA2_CONDITION_PIN  12  // Indicates lean (LOW) or rich (HIGH) for subgroup 2
#define LAMBDA2_ENABLE_PIN     13  // Enable signal for lambda 2 condition

// Desired target voltages for lean and rich conditions
#define LEAN_TARGET          0.45  // V for slightly lean condition
#define RICH_TARGET          0.65  // V for slightly rich condition

// DAC output constraints
#define DAC_MIN              0
#define DAC_MAX              255
#define DAC_MID              128

// Proportional gain for the controller
#define KP                   2.0

// Threshold for enabling the lambda probe emulation output
#define ERROR_THRESHOLD      0.05  // Voltage difference threshold

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Setup analog inputs and DAC outputs
  analogReadResolution(12);  // ESP32 default ADC resolution is 12-bit (0-4095)
  analogSetAttenuation(ADC_11db);  // Full-scale voltage range (0-3.3V)

  // Setup digital pins for lambda probe emulation
  pinMode(LAMBDA1_CONDITION_PIN, INPUT);
  pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  pinMode(LAMBDA2_CONDITION_PIN, INPUT);
  pinMode(LAMBDA2_ENABLE_PIN, INPUT);
}

void loop() {
  // Read the current injection pulse length from the injection computer
  int injectionPulseRaw = analogRead(INJECTION_PULSE_PIN);
  float injectionPulseVoltage = injectionPulseRaw * (3.3 / 4095.0);

  // Read lambda sensors
  int lambda1Raw = analogRead(LAMBDA1_PIN);
  int lambda2Raw = analogRead(LAMBDA2_PIN);
  float lambda1Voltage = lambda1Raw * (3.3 / 4095.0);
  float lambda2Voltage = lambda2Raw * (3.3 / 4095.0);

  // Calculate the error for each subgroup
  float error1 = LEAN_TARGET - lambda1Voltage;  // Subgroup 1 should be lean
  float error2 = RICH_TARGET - lambda2Voltage;  // Subgroup 2 should be rich

  // Calculate the offset
  int offset1 = DAC_MID + (int)(KP * error1 * 128.0);
  int offset2 = DAC_MID + (int)(KP * error2 * 128.0);

  // Constrain the offset values to DAC limits
  offset1 = constrain(offset1, DAC_MIN, DAC_MAX);
  offset2 = constrain(offset2, DAC_MIN, DAC_MAX);

  // Output the offset values via DACs
  dacWrite(DAC1_PIN, offset1);
  dacWrite(DAC2_PIN, offset2);

  // Handle lambda probe emulation for subgroup 1
  if (abs(error1) > ERROR_THRESHOLD) {
    // Set the condition pin based on lean or rich status
    pinMode(LAMBDA1_CONDITION_PIN, OUTPUT);
    if (lambda1Voltage < LEAN_TARGET) {
      digitalWrite(LAMBDA1_CONDITION_PIN, LOW);  // Lean condition
    } else {
      digitalWrite(LAMBDA1_CONDITION_PIN, HIGH);  // Rich condition
    }
    // Enable the condition output
    pinMode(LAMBDA1_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA1_ENABLE_PIN, HIGH);
  } else {
    // Disable the output by setting pins as input
    pinMode(LAMBDA1_CONDITION_PIN, INPUT);
    pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  }

  // Handle lambda probe emulation for subgroup 2
  if (abs(error2) > ERROR_THRESHOLD) {
    // Set the condition pin based on lean or rich status
    pinMode(LAMBDA2_CONDITION_PIN, OUTPUT);
    if (lambda2Voltage < LEAN_TARGET) {
      digitalWrite(LAMBDA2_CONDITION_PIN, LOW);  // Lean condition
    } else {
      digitalWrite(LAMBDA2_CONDITION_PIN, HIGH);  // Rich condition
    }
    // Enable the condition output
    pinMode(LAMBDA2_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA2_ENABLE_PIN, HIGH);
  } else {
    // Disable the output by setting pins as input
    pinMode(LAMBDA2_CONDITION_PIN, INPUT);
    pinMode(LAMBDA2_ENABLE_PIN, INPUT);
  }

  // Debugging output
  Serial.print("Lambda1: "); Serial.print(lambda1Voltage); Serial.print("V, Offset1: "); Serial.println(offset1);
  Serial.print("Lambda2: "); Serial.print(lambda2Voltage); Serial.print("V, Offset2: "); Serial.println(offset2);

  // Delay to allow time for the engine's conditions to stabilize
  delay(100);
}
```

### Improvements in the Code:

1. **Lambda Condition and Enable Pins:**
   - **Lambda Condition Pin:** Indicates whether the engine is running lean (`LOW`) or rich (`HIGH`) for each subgroup.
   - **Enable Pin:** This pin is set to `OUTPUT` mode and driven high only if the error magnitude is greater than a specified threshold (`ERROR_THRESHOLD`). Otherwise, it is set to `INPUT` mode to disable output.

2. **Threshold-Based Output Control:**
   - If the absolute error exceeds the threshold, the lambda condition is output, and the enable pin is activated.
   - If the error is within the acceptable range, the pins are set to `INPUT` mode to effectively disable the output, simulating that the lambda probe is not reporting an extreme lean or rich condition.

3. **Emulation of Lambda Probe Output:**
   - The emulation mimics the behavior of a narrowband lambda sensor, which switches between lean and rich signals depending on the AFR (Air-Fuel Ratio).
   - This helps ensure that the engine's control unit interprets the signals correctly and adjusts fueling accordingly.

4. **Error Handling:**
   - The code automatically disables the outputs if the error is small, preventing unnecessary toggling between lean/rich states when the engine is operating within desired limits.

This approach allows for a more accurate simulation of the narrowband lambda sensor behavior, ensuring that the engine control unit (ECU) receives appropriate signals based on the modified AFR conditions for each subgroup.

improve the lambda output emulation by determining the magnitude and direction of error by the offset drift - if there is assymetry of the offsets, enable the first lambda output pin with the signal representing the direction of the drift (lean or rich). The second lambda emulation output should be enabled and set to indicate if limit exceeded in lean or rich direction if any if the errors generating offsets exceeds the DAC limit . 

To further enhance the lambda output emulation, we will adjust the logic to account for the asymmetry between the offsets and the limits exceeded in either direction. Here's a breakdown of the new implementation:

1. **Direction of Drift (Asymmetry):**
   - **Lambda Condition Pin 1:** Indicates the direction of the offset drift (lean or rich) if there is asymmetry between the two subgroup offsets. If one offset is greater than the other, this indicates a drift toward either lean or rich conditions.
   - **Enable Pin 1:** This pin is activated if there is a noticeable asymmetry between the two offsets.

2. **Limit Exceeded Condition:**
   - **Lambda Condition Pin 2:** Indicates if the offset has reached or exceeded the DAC limits (lean or rich).
   - **Enable Pin 2:** This pin is activated if any offset reaches the DAC limit, indicating a significant error.

### Revised Arduino Code:

```cpp
// Pin definitions
#define INJECTION_PULSE_PIN  34  // Analog input from the injection computer
#define LAMBDA1_PIN          35  // Analog input from lambda sensor 1
#define LAMBDA2_PIN          36  // Analog input from lambda sensor 2
#define DAC1_PIN             25  // DAC output for subgroup 1
#define DAC2_PIN             26  // DAC output for subgroup 2

// Digital output pins for narrowband lambda probe emulation
#define LAMBDA1_CONDITION_PIN  27  // Direction of drift (lean/rich)
#define LAMBDA1_ENABLE_PIN     14  // Enable signal for drift indication
#define LAMBDA2_CONDITION_PIN  12  // Limit exceeded condition (lean/rich)
#define LAMBDA2_ENABLE_PIN     13  // Enable signal for limit exceeded

// Desired target voltages for lean and rich conditions
#define LEAN_TARGET          0.45  // V for slightly lean condition
#define RICH_TARGET          0.65  // V for slightly rich condition

// DAC output constraints
#define DAC_MIN              0
#define DAC_MAX              255
#define DAC_MID              128

// Proportional gain for the controller
#define KP                   2.0

// Threshold for enabling the lambda probe emulation output
#define ERROR_THRESHOLD      0.05  // Voltage difference threshold for asymmetry detection
#define ASYMMETRY_THRESHOLD  10    // Threshold difference between offsets to trigger drift condition

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Setup analog inputs and DAC outputs
  analogReadResolution(12);  // ESP32 default ADC resolution is 12-bit (0-4095)
  analogSetAttenuation(ADC_11db);  // Full-scale voltage range (0-3.3V)

  // Setup digital pins for lambda probe emulation
  pinMode(LAMBDA1_CONDITION_PIN, INPUT);
  pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  pinMode(LAMBDA2_CONDITION_PIN, INPUT);
  pinMode(LAMBDA2_ENABLE_PIN, INPUT);
}

void loop() {
  // Read the current injection pulse length from the injection computer
  int injectionPulseRaw = analogRead(INJECTION_PULSE_PIN);
  float injectionPulseVoltage = injectionPulseRaw * (3.3 / 4095.0);

  // Read lambda sensors
  int lambda1Raw = analogRead(LAMBDA1_PIN);
  int lambda2Raw = analogRead(LAMBDA2_PIN);
  float lambda1Voltage = lambda1Raw * (3.3 / 4095.0);
  float lambda2Voltage = lambda2Raw * (3.3 / 4095.0);

  // Calculate the error for each subgroup
  float error1 = LEAN_TARGET - lambda1Voltage;  // Subgroup 1 should be lean
  float error2 = RICH_TARGET - lambda2Voltage;  // Subgroup 2 should be rich

  // Calculate the offset
  int offset1 = DAC_MID + (int)(KP * error1 * 128.0);
  int offset2 = DAC_MID + (int)(KP * error2 * 128.0);

  // Constrain the offset values to DAC limits
  offset1 = constrain(offset1, DAC_MIN, DAC_MAX);
  offset2 = constrain(offset2, DAC_MIN, DAC_MAX);

  // Output the offset values via DACs
  dacWrite(DAC1_PIN, offset1);
  dacWrite(DAC2_PIN, offset2);

  // Handle lambda probe emulation for asymmetry drift (Lambda1)
  if (abs(offset1 - offset2) > ASYMMETRY_THRESHOLD) {
    // Set the condition pin based on the direction of drift (which offset is higher)
    pinMode(LAMBDA1_CONDITION_PIN, OUTPUT);
    if (offset1 > offset2) {
      digitalWrite(LAMBDA1_CONDITION_PIN, HIGH);  // Drift towards rich
    } else {
      digitalWrite(LAMBDA1_CONDITION_PIN, LOW);   // Drift towards lean
    }
    // Enable the condition output
    pinMode(LAMBDA1_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA1_ENABLE_PIN, HIGH);
  } else {
    // Disable the output by setting pins as input
    pinMode(LAMBDA1_CONDITION_PIN, INPUT);
    pinMode(LAMBDA1_ENABLE_PIN, INPUT);
  }

  // Handle lambda probe emulation for limit exceeded (Lambda2)
  bool limitExceeded = (offset1 == DAC_MIN || offset1 == DAC_MAX || offset2 == DAC_MIN || offset2 == DAC_MAX);
  if (limitExceeded) {
    // Set the condition pin based on which limit was exceeded
    pinMode(LAMBDA2_CONDITION_PIN, OUTPUT);
    if (offset1 == DAC_MAX || offset2 == DAC_MAX) {
      digitalWrite(LAMBDA2_CONDITION_PIN, HIGH);  // Rich limit exceeded
    } else {
      digitalWrite(LAMBDA2_CONDITION_PIN, LOW);   // Lean limit exceeded
    }
    // Enable the condition output
    pinMode(LAMBDA2_ENABLE_PIN, OUTPUT);
    digitalWrite(LAMBDA2_ENABLE_PIN, HIGH);
  } else {
    // Disable the output by setting pins as input
    pinMode(LAMBDA2_CONDITION_PIN, INPUT);
    pinMode(LAMBDA2_ENABLE_PIN, INPUT);
  }

  // Debugging output
  Serial.print("Lambda1: "); Serial.print(lambda1Voltage); Serial.print("V, Offset1: "); Serial.println(offset1);
  Serial.print("Lambda2: "); Serial.print(lambda2Voltage); Serial.print("V, Offset2: "); Serial.println(offset2);

  // Delay to allow time for the engine's conditions to stabilize
  delay(100);
}
```

### Code Improvements:
1. **Asymmetry Detection and Drift Indication:**
   - If the offsets for the two subgroups differ by more than the `ASYMMETRY_THRESHOLD`, the system will determine the direction of the drift.
   - **Lambda1 Condition Pin** is set based on which offset is higher, representing either a lean or rich drift.
   - **Lambda1 Enable Pin** is activated if there is significant asymmetry.

2. **Limit Exceeded Detection:**
   - **Lambda2 Condition Pin** indicates if the DAC output has reached its maximum or minimum limit, representing either a rich or lean condition.
   - **Lambda2 Enable Pin** is activated if any of the offsets hit the DAC limits, signifying that the controller is operating at its boundary.

3. **Pin Configuration:**
   - The condition and enable pins are set to `OUTPUT` when they need to actively indicate a state. They revert to `INPUT` when no indication is necessary, effectively disabling the output.

4. **Threshold and Gain Adjustments:**
   - **ASYMMETRY_THRESHOLD** and **KP** can be adjusted based on the engine's behavior and desired sensitivity.

### Summary:
This setup emulates the behavior of narrowband lambda probes more accurately by incorporating the direction of the error (drift) and detecting when the controller is at its operational limits. This logic allows the ECU to interpret the signals and adjust fueling accordingly while providing additional safeguards and diagnostics.

