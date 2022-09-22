#include "ph_sensor.hpp"
#include "adc.hpp"
#include "hw_definitions.hpp"

void phsensor_setup();
int16_t phsensor_read();

//****************
// Implementation.
//****************

void phsensor_setup()
{
    adc_commonSetup();
}

int16_t phsensor_read()
{
    constexpr uint16_t aReadResolution = 1024;
    constexpr int16_t offsetAnalogRead = 205; // Around 1V.
    constexpr uint8_t maxReadings = 10;

    uint16_t total = 0;

    // Compute average of up to maxReadings.
    for (uint8_t i = 0; i < maxReadings; i++)
    {
        uint16_t val = adc_measure(ph);
        total += val;
    }

    uint16_t avg = total / maxReadings;

    int16_t ph = ((aReadResolution - avg) + offsetAnalogRead) / 10 * PH_MAGIC_NUMBER;

    if (ph < 0 || ph > 14000)
        return -1;

    return ph;
}