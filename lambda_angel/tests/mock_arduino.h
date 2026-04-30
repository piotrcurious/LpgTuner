#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include <cstdint>

typedef std::string String;

#define PI 3.14159265358979323846
#define FALLING 0

unsigned long millis();
void delay(unsigned long ms);
int analogRead(int pin);
void noInterrupts();
void interrupts();

#define Arduino_h
#endif
