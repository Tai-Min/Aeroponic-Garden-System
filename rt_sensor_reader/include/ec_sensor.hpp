#pragma once
#include <stdint.h>

/**
 * @brief Setup hardware required for EC sensor.
 */
void ecsensor_setup();

/**
 * @brief Read EC sensor value in ppm.
 * Ambient temperature assumed to 25 degree Celcius.
 */
int16_t ecsensor_read();