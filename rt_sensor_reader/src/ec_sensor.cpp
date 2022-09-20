#include "ec_sensor.hpp"
#include "adc.hpp"
#include "hw_definitions.hpp"

void ecsensor_setup();
int16_t ecsensor_read();

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

void ecsensor_setup()
{
    adc_commonSetup();
}

int16_t ecsensor_read()
{
    constexpr uint8_t vRefVolts = 5;
    constexpr uint16_t aReadResolution = 1024;
    constexpr float kValue = 0.67; // Per https://www.omnicalculator.com/chemistry/tds
    uint16_t measurement = readAvg();

    // Per https://github.com/DFRobot/GravityTDS/blob/master/GravityTDS.cpp
    // not good source though? As there is clear division by 0 in the source.
    float asVoltage = (float)measurement * (float)vRefVolts / (float)aReadResolution;

    // Assume that ambient temp is already 25 degrees Celsius.
    float ec25 = asVoltage * (133.42 * asVoltage * asVoltage - 255.86 * asVoltage + 857.39) * kValue;
    int16_t tds = ec25 * 0.5;

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