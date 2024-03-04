// A function that takes injection length and rpm values as parameters and returns the fuel consumption in liters per hour
float fuelComputer(float injectionLength, int rpm) {
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
  // Return the result of applying the fuel consumption lambda expression to the injection length and rpm parameters
  return fuelConsumption(injectionLength, rpm);
}
