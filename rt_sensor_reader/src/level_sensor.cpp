#include "level_sensor.hpp"

#include <util/delay.h>
#include <avr/io.h>
#include "timer.hpp"

void usensor_setup();
int16_t usensor_read(const LevelSensorHardware &sensor);

/**
 * @brief Perform single measurement.
 * @param sensor Sensor to read.
 * @return Level in milimeters or -1 if fail.
 */
int16_t readSingle(const LevelSensorHardware &sensor);

//****************
// Implementation.
//****************

#define SET_OUTPUT(REGISTER, REGISTER_BIT) *REGISTER |= (1 << REGISTER_BIT)
#define SET_INPUT(REGISTER, REGISTER_BIT) *REGISTER &= ~(1 << REGISTER_BIT)

#define SET_LEVEL(PORT, PORT_BIT, LEVEL) LEVEL ? *PORT |= (1 << PORT_BIT) : *PORT &= ~(1 << PORT_BIT)
#define READ_LEVEL(PIN, PIN_BIT) (*PIN & (1 << PIN_BIT))

void usensor_setup()
{
    timer_setup();
}

int16_t usensor_read(const LevelSensorHardware &sensor)
{
    constexpr uint8_t numReadings = 10;

    uint8_t iter = 0;
    uint32_t numerator = 0;
    while (iter < numReadings)
    {
        int16_t val = readSingle(sensor);
        if (val < 0)
            continue;

        numerator += val;
        iter++;
    }

    return numerator / numReadings;
}

int16_t readSingle(const LevelSensorHardware &sensor)
{
    uint32_t startUs;
    uint32_t stopUs;
    constexpr uint32_t toutUs = 50000;

    SET_OUTPUT(sensor.reg, sensor.regBit);
    SET_LEVEL(sensor.port, sensor.portBit, 1);
    _delay_us(11);
    SET_LEVEL(sensor.port, sensor.portBit, 0);
    SET_INPUT(sensor.reg, sensor.regBit);

    timer_reset();
    startUs = timer_read();
    stopUs = startUs;

    while (!READ_LEVEL(sensor.pin, sensor.pinBit))
    {
        startUs = timer_read();
        if (startUs - stopUs > toutUs)
            return -1;
    }

    stopUs = startUs;

    while (READ_LEVEL(sensor.pin, sensor.pinBit))
    {
        stopUs = timer_read();
        if (stopUs - startUs > toutUs)
            return -1;
    }

    uint32_t stamp = stopUs - startUs;
    uint32_t distanceMilimeters = stamp * 343 / 2000;

    if (distanceMilimeters > 4000)
        return -1;

    return distanceMilimeters;
}