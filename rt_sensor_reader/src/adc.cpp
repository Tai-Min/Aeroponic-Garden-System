#include "adc.hpp"

void adc_commonSetup();
uint16_t adc_measure(uint8_t pin);
int16_t adc_temperature();

namespace
{
    constexpr uint8_t tempSensor = 255; //!< Magic value to access Attiny85 internal temperature sensor.

    /**
     * @brief Convert pin to it's mask in ADMUX.
     * Only physical pins on Attiny85 can be used in this function,
     * otherwise mask tied to GND will be returned.
     */
    uint8_t adc_pinToMask(uint8_t pin);
}

//****************
// Implementation.
//****************

void adc_commonSetup()
{
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADEN);
}

uint16_t adc_measure(uint8_t pin)
{
    constexpr uint8_t clearMask = ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));

    ADMUX &= clearMask;          // Clear mux bits.
    ADMUX |= adc_pinToMask(pin); // Set correct pin.

    // Start conversion.
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
        ;

    // Per documentation: ADCL must be read first, then ADCH.
    uint8_t l = ADCL;
    uint8_t h = ADCH;
    return (h << 8) | l;
}

// Not working correctly as of yet?
/*int16_t adc_temperature()
{
    constexpr int32_t a10 = 92;
    constexpr int32_t b10 = -25357;

    ADMUX |= (1 << REFS1); // 1.1V reference.
    uint16_t adcVal = adc_measure(tempSensor);
    ADMUX &= ~((1 << REFS2) | (1 << REFS1) | (1 << REFS0)); // Restore VCC reference.

    int32_t temp = (a10 * adcVal + b10) / 100;

    return temp;
}*/

namespace
{
    uint8_t adc_pinToMask(uint8_t pin)
    {
        return pin == PB2   ? (1 << MUX0)
               : pin == PB3 ? (1 << MUX1) | (1 << MUX0)
               : pin == PB4 ? (1 << MUX1)
               : pin == PB5 ? 0
               : pin == 255 ? (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0) // Temp sensor.
                            : (1 << MUX3) | (1 << MUX2) | (1 << MUX0);              // Default to GND.
    }
}
