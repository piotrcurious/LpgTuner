// A function to integrate pulses representing fuel injection impulses
// The buffer array consists of two parts: pulse lengths and rpm values
// The function reports integrated fuel consumption over a time window
// The function infers spacing of the pulses from rpm values

// Parameters:
// buffer: a two-dimensional array of pulse lengths and rpm values
// size: the size of the buffer array
// window: the time window in seconds
// Returns: the integrated fuel consumption in milliliters

float integratePulses(float buffer[][2], int size, float window) {
  // Initialize variables
  float consumption = 0; // the integrated fuel consumption
  float time = 0; // the current time
  float interval = 0; // the time interval between pulses
  float volume = 0; // the volume of fuel injected per pulse
  float density = 0.75; // the density of fuel in g/ml
  int i = 0; // the index of the buffer array

  // Loop through the buffer array
  while (i < size && time < window) {
    // Calculate the time interval between pulses from rpm values
    // rpm = 60 / (interval * 2)
    // interval = 30 / rpm
    interval = 30 / buffer[i][1];

    // Calculate the volume of fuel injected per pulse from pulse lengths
    // volume = pulse length * flow rate
    // Assume a constant flow rate of 10 ml/s
    volume = buffer[i][0] * 10;

    // Update the integrated fuel consumption
    // consumption = consumption + volume * density
    consumption = consumption + volume * density;

    // Update the current time
    // time = time + interval
    time = time + interval;

    // Increment the index
    i++;
  }

  // Return the integrated fuel consumption
  return consumption;
}
