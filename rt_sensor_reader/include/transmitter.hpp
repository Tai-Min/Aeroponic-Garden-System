#pragma once
#include <stdint.h>

/**
 * @brief Setup hardware for serial like transmitter.
 */
void transmitter_setup(uint32_t baud = 9600);

/**
 * @brief Send one measurement.
 * @param prefix Prefix of measurement, i.e "ec", "ph", "u0" etc.
 * @param val Measurement.
 */
void transmitter_sendMeasurement(const char* prefix, int16_t val);