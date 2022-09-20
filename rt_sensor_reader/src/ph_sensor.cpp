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
    constexpr uint8_t maxReadings = 10;

    uint32_t total = 0;

    // Compute average of up to maxReadings.
    for (uint8_t i = 0; i < maxReadings; i++)
    {
        uint16_t val = adc_measure(ph);
        total += val;
    }

    uint16_t avg = total / maxReadings;

    return avg * (14000 / aReadResolution); // To 0 - 14000 milli pH range.
}