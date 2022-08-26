#pragma once
#include <stdint.h>

/**
 * @brief Setup timer hardware.
 */
void timer_setup();

/**
 * @brief Reset timer.
 */
void timer_reset();

/**
 * @brief Read timer's current value.
 */
uint32_t timer_read();