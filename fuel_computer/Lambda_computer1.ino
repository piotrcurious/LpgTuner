// A function that takes injection length and rpm values as parameters and returns the fuel consumption in liters per hour
float fuelComputer(float injectionLength, int rpm) {
  if (rpm <= 0) return 0;

  // --- Configuration ---
  const float FLOW_RATE_L_PER_MIN = 0.5f;
  const int NUM_CYLINDERS = 4;
  // ---------------------

  // A lambda expression that calculates the time in seconds for one engine revolution from the rpm value
  auto getTimePerRev = [](int r) -> float {
    return 60.0f / r;
  };

  // A lambda expression that calculates the fuel volume in liters for a single injection
  auto getFuelVolumePerInjection = [FLOW_RATE_L_PER_MIN](float il) -> float {
    const float FLOW_RATE_L_PER_MS = FLOW_RATE_L_PER_MIN / 60000.0f;
    return il * FLOW_RATE_L_PER_MS;
  };

  // A lambda expression that calculates the fuel consumption in liters per hour from the time and fuel volume values
  auto getFuelConsumptionLPerHour = [getTimePerRev, getFuelVolumePerInjection, NUM_CYLINDERS](float il, int r) -> float {
    float secondsPerCycle = 2.0f * getTimePerRev(r); // 4-stroke engine: 1 injection every 2 revolutions per cylinder
    float litersPerSecondPerCylinder = getFuelVolumePerInjection(il) / secondsPerCycle;
    float totalLitersPerHour = (litersPerSecondPerCylinder * 3600.0f) * NUM_CYLINDERS;
    return totalLitersPerHour;
  };

  // Return the result of applying the fuel consumption lambda expression to the injection length and rpm parameters
  return getFuelConsumptionLPerHour(injectionLength, rpm);
}
