#pragma once
#include <stdint.h>

/**
 * @brief Setup hardware required for pH sensor.
 */
void phsensor_setup();

/**
 * @brief Get milli pH value.
 * No temperature compensation implemented.
 */
int16_t phsensor_read();