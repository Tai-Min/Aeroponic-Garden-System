#include "level_sensor.hpp"

#include <util/delay.h>
#include <avr/io.h>
#include "timer.hpp"

void usensor_setup();
int16_t usensor_read(uint8_t sensor);

namespace
{
    /**
     * @brief Set pin as output.
     */
    inline void setOutput(uint8_t pin);

    /**
     * @brief Set pin as input.
     */
    inline void setInput(uint8_t pin);

    /**
     * @brief Set level of output pin.
     */
    inline void setLevel(uint8_t pin, bool level);

    /**
     * @brief Read level of input pin.
     */
    inline uint8_t readLevel(uint8_t pin);

    /**
     * @brief Perform single measurement.
     * @param sensor Sensor to read.
     * @return Level in milimeters or -1 if fail.
     */
    int16_t readSingle(uint8_t sensor);
}

//****************
// Implementation.
//****************

void usensor_commonSetup()
{
    timer_setup();
}

int16_t usensor_read(uint8_t sensor)
{
    constexpr uint8_t maxReadings = 10;

    uint32_t total = 0;
    uint8_t performedReadings = 0;

    // Compute average of up to maxReadings.
    for (uint8_t i = 0; i < maxReadings; i++)
    {
        int16_t val = readSingle(sensor);
        if (val < 0)
            continue;

        total += val;
        performedReadings++;
    }

    if (!performedReadings)
        return -1;

    return total / performedReadings;
}

namespace
{

    inline void setOutput(uint8_t pin)
    {
        DDRB |= (1 << pin);
    }

    inline void setInput(uint8_t pin)
    {
        DDRB &= ~(1 << pin);
    }

    inline void setLevel(uint8_t pin, bool level)
    {
        level ? PORTB |= (1 << pin) : PORTB &= ~(1 << pin);
    }

    inline uint8_t readLevel(uint8_t pin)
    {
        return (PINB & (1 << pin));
    }

    int16_t readSingle(uint8_t sensor)
    {
        uint32_t startUs;
        uint32_t stopUs;
        constexpr uint32_t toutUs = 40000;

        // Trigger.
        setOutput(sensor);
        setLevel(sensor, 1);
        _delay_us(11);
        setLevel(sensor, 0);
        setInput(sensor);

        // Wait for pin to go up (start measure distance).
        timer_reset();
        startUs = timer_read();
        stopUs = startUs;
        while (!readLevel(sensor))
        {
            startUs = timer_read();
            if (startUs - stopUs > toutUs)
                return -1;
        }

        // Wait for pin to go low (stop measure distance).
        stopUs = startUs;
        while (readLevel(sensor))
        {
            stopUs = timer_read();
            if (stopUs - startUs > toutUs)
                return -1;
        }

        // Compute time of flight to distance.
        uint32_t stamp = stopUs - startUs;
        uint32_t distanceMilimeters = stamp * 346 / 2000; // Using 25 degrees Celsius reference.

        if (distanceMilimeters > 4000)
            return -1;

        return distanceMilimeters;
    }
}