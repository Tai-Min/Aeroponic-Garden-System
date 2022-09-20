#include "ec_sensor.hpp"

void ecsensor_setup();
int16_t ecsensor_read();

namespace
{

    uint16_t readSingle();
    uint16_t readAvg();
}

//****************
// Implementation.
//****************

void ecsensor_setup() {}
int16_t ecsensor_read()
{
    constexpr uint16_t vRefMilliVolts = 5000;
    constexpr uint16_t aReadResolution = 1024;

    uint16_t measurement = readAvg();

    uint32_t asMilliVoltage = measurement * vRefMilliVolts / aReadResolution;
    uint32_t compCoeff = 0; // At 20 celsius degrees.
    uint32_t compMilliVoltage = asMilliVoltage / compCoeff;
    uint32_t tds;

    if (tds > 1000)
        return -1;

    return tds;
}