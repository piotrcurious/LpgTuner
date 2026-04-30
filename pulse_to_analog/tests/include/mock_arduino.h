#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define IRAM_ATTR
#define ICACHE_RAM_ATTR

typedef enum {
    DAC_CHANNEL_1 = 0,
    DAC_CHANNEL_2 = 1,
} dac_channel_t;

typedef enum {
    INPUT,
    OUTPUT,
    CHANGE
} pinMode_t;

// Mocks
extern uint32_t mock_ccount;
extern uint32_t mock_gpio_in;
extern uint8_t mock_dac_val[2];

void dac_output_enable(dac_channel_t channel);
void dac_output_voltage(dac_channel_t channel, uint8_t voltage);
void pinMode(int pin, pinMode_t mode);
void attachInterrupt(int interrupt, void (*userFunc)(void), int mode);
int digitalPinToInterrupt(int pin);
uint32_t getCpuFrequencyMhz();
uint32_t REG_READ(uint32_t reg);

#define GPIO_IN_REG 0x3FF4403C

#endif
