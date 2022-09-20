#pragma once
#include <stdint.h>

/**
 * @brief Setup hardware for serial like transmitter. Sets up serial as 9600 N 8 1.
 */
void transmitter_setup();

/**
 * @brief Send one measurement.
 * @param prefix Prefix of measurement, i.e "ec", "ph", "u0" etc.
 * @param val Measurement.
 */
void transmitter_sendMeasurement(const char *prefix, int16_t val);