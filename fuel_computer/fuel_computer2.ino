// A function that takes injection length, rpm, and km/h values as parameters and returns the fuel consumption in liters per 100 km
float fuelComputer2(float injectionLength, int rpm, float kmh) {
  // A lambda expression that calculates the time in seconds from the rpm value
  auto time =  -> float {
    return 60.0 / rpm;
  };
  // A lambda expression that calculates the fuel volume in milliliters from the injection length value
  auto fuelVolume =  -> float {
    return injectionLength * 0.001;
  };
  // A lambda expression that calculates the fuel consumption in liters per hour from the time and fuel volume values
  auto fuelConsumption = time, fuelVolume -> float {
    return (fuelVolume(injectionLength) / time(rpm)) * 3600.0 / 1000.0;
  };
  // A lambda expression that calculates the fuel consumption in liters per 100 km from the fuel consumption in liters per hour and the km/h values
  auto fuelConsumption2 = fuelConsumption -> float {
    return (fuelConsumption(injectionLength, rpm) / kmh) * 100.0;
  };
  // Return the result of applying the fuel consumption2 lambda expression to the injection length, rpm, and km/h parameters
  return fuelConsumption2(injectionLength, rpm, kmh);
}
