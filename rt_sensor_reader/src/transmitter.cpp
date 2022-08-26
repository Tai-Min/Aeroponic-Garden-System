#include "transmitter.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>

void transmitter_setup(uint32_t baud);
void transmitter_sendMeasurement(const char *prefix, int16_t val);

/**
 * @brief Convert value to string.
 * @param val Value to convert.
 * @param buf Output buffer that can contain at most 6 characters.
 */
static void toString(int16_t val, char *buf);

//****************
// Implementation.
//****************

namespace
{
    bool isSetup = false;
}

void transmitter_setup(uint32_t baud)
{
    if (isSetup)
        return;

    cli();
    uint16_t baudReg = F_CPU / 16 / baud - 1;

    UBRR0H = (baudReg >> 8);
    UBRR0L = (baudReg);

    UCSR0C = 0x06; // 8N1
    UCSR0B = (1 << TXEN0);
    sei();
}

void transmitter_sendMeasurement(const char *prefix, int16_t val)
{
    char b[11]; // minus symbol + 5 max digits + asteriks + max 3 digits + terminator

    toString(val, b);

    uint8_t i = 0;
    uint8_t checksum = 0;
    while (b[i] != '\0')
    {
        while (!(UCSR0A & (1 << UDRE0)))
            ;
        checksum ^= b[i++];
    }
    b[i++] = '*';

    toString(checksum, b + i);

    i = 0;
    while (prefix[i] != '\0')
    {
        while (!(UCSR0A & (1 << UDRE0)))
            ;
        UDR0 = prefix[i++];
    }

    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = ':';

    i = 0;
    while (b[i] != '\0')
    {
        while (!(UCSR0A & (1 << UDRE0)))
            ;
        UDR0 = b[i++];
    }

    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = '\n';
}

static void toString(int16_t val, char *buf)
{

    uint16_t divider = 1;
    uint8_t idxCntr = 0;

    if (val < 0)
    {
        buf[idxCntr++] = '-';
        val *= -1;
    }

    int16_t valTmp = val;

    while (valTmp >= 10)
    {
        divider *= 10;
        valTmp /= 10;
    }

    while (divider)
    {
        uint8_t digitInt = val / divider;
        val -= digitInt * divider;
        divider /= 10;

        buf[idxCntr++] = digitInt + '0';
    }

    buf[idxCntr] = '\0';
}