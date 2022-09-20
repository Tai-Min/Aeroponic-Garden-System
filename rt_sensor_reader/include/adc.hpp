#pragma once
#include <stdint.h>
#include <avr/io.h>

/**
 * @brief Setup common ADC hardware.
 */
void adc_commonSetup();

/**
 * @brief Get ADC value from given pin.
 */
uint16_t adc_measure(uint8_t pin);

/**
 * @brief Get temperature in degrees Celsius from
 * internal temperature sensor. Not too accurate.
 */
// int16_t adc_temperature();