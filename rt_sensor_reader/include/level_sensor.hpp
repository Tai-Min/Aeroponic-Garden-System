#pragma once
#include <stdint.h>

/**
 * @brief Contains info about hardware for single ultrasonic sensor.
 */
struct LevelSensorHardware
{
    volatile uint8_t *reg;
    uint8_t regBit;
    volatile uint8_t *port;
    uint8_t portBit;
    volatile uint8_t *pin;
    uint8_t pinBit;
};

/**
 * @brief Setup ultrasonic level sensor's common hardware.
 */
void usensor_setup();

/**
 * @brief Read level of ultrasonic level sensor in milimeters.
 * @param sensor Sensor to read.
 * @return Level in milimeters or -1 if fail.
 */
int16_t usensor_read(const LevelSensorHardware &sensor);