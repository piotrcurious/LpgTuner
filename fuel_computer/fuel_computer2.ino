// A function that takes injection length, rpm, and km/h values as parameters and returns the fuel consumption in liters per 100 km
float fuelComputer2(float injectionLength, int rpm, float kmh) {
  if (kmh <= 0) return 0;
  if (rpm <= 0) return 0;

  // A lambda expression that calculates the time in seconds for one engine revolution from the rpm value
  auto getTimePerRev = [](int r) -> float {
    return 60.0f / r;
  };

  // A lambda expression that calculates the fuel volume in liters for a single injection
  // Assuming a standard flow rate of 0.5 L/min = 0.5 / 60000 L/ms
  auto getFuelVolumePerInjection = [](float il) -> float {
    const float FLOW_RATE_L_PER_MS = 0.5f / 60000.0f;
    return il * FLOW_RATE_L_PER_MS;
  };

  // A lambda expression that calculates the fuel consumption in liters per hour from the time and fuel volume values
  auto getFuelConsumptionLPerHour = [getTimePerRev, getFuelVolumePerInjection](float il, int r) -> float {
    float secondsPerCycle = 2.0f * getTimePerRev(r); // 4-stroke engine: 1 injection every 2 revolutions
    float litersPerSecond = getFuelVolumePerInjection(il) / secondsPerCycle;
    return litersPerSecond * 3600.0f;
  };

  // A lambda expression that calculates the fuel consumption in liters per 100 km from the fuel consumption in liters per hour and the km/h values
  auto getFuelEconomyL100km = [getFuelConsumptionLPerHour](float il, int r, float v) -> float {
    return (getFuelConsumptionLPerHour(il, r) / v) * 100.0f;
  };

  // Return the result of applying the fuel consumption2 lambda expression to the injection length, rpm, and km/h parameters
  return getFuelEconomyL100km(injectionLength, rpm, kmh);
}
