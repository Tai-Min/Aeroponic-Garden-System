#pragma once
#include <stdint.h>

struct SerialHardware
{
#ifdef UNO_DEBUG
#else
    volatile uint8_t *reg;
    uint8_t regBit;
    volatile uint8_t *port;
    uint8_t portBit;
#endif
};

/**
 * @brief Setup hardware for serial like transmitter. Sets up serial as 9600 N 8 1.
 * @param hw Hardware definition for serial.
 */
void transmitter_setup(const SerialHardware &hw);

/**
 * @brief Send one measurement.
 * @param prefix Prefix of measurement, i.e "ec", "ph", "u0" etc.
 * @param val Measurement.
 */
void transmitter_sendMeasurement(const char *prefix, int16_t val);