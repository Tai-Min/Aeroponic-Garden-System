#include "tds_sensor.hpp"
#include "adc.hpp"
#include "hw_definitions.hpp"

void tdssensor_setup();
int16_t tdssensor_read();

namespace
{
    /**
     * @brief Perform multiple analog reads and return it's average.
     */
    uint16_t readAvg();
}

//****************
// Implementation.
//****************

void tdssensor_setup()
{
    adc_commonSetup();
}

int16_t tdssensor_read()
{
    // Tuned by hand.
    // I don't trust Arduino source code of this sensor
    // due to divides by 0 in the code.
    // Tuned at 25 celsius degrees.
    constexpr int32_t a = 9580;
    constexpr int32_t b = -1366;

    constexpr uint8_t vRefVolts = 5;
    constexpr uint16_t aReadResolution = 1024;

    uint16_t measurement = readAvg();

    int16_t tds = (measurement * vRefVolts * a / aReadResolution + b) / 10;

    if (tds > 1000)
        return -1;

    return tds;
}

namespace
{

    uint16_t readAvg()
    {
        constexpr uint8_t maxReadings = 10;

        uint32_t total = 0;

        // Compute average of up to maxReadings.
        for (uint8_t i = 0; i < maxReadings; i++)
        {
            uint16_t val = adc_measure(ec);
            total += val;
        }

        return total / maxReadings;
    }
}