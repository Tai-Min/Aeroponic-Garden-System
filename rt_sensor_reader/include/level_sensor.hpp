#pragma once
#include <stdint.h>

/**
 * @brief Setup ultrasonic level sensor's common hardware.
 */
void usensor_commonSetup();

/**
 * @brief Read level of ultrasonic level sensor in milimeters.
 * Ambient temperature assumed to 25 degrees Celsius.
 * @param sensor Sensor to read.
 * @return Level in milimeters or -1 if fail.
 */
int16_t usensor_read(uint8_t sensor);