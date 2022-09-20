#include "transmitter.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include "hw_definitions.hpp"

void transmitter_setup();
void transmitter_sendMeasurement(const char *prefix, int16_t val);

//****************
// Implementation.
//****************

namespace
{
    /**
     * @brief Convert value to string.
     * @param val Value to convert.
     * @param buf Output buffer that can contain at most 6 characters.
     */
    void toString(int16_t val, char *buf);

    /**
     * @brief Write single byte via soft serial.
     */
    void writeByte(char b);
}

void transmitter_setup()
{
    DDRB |= (1 << serial); // Tx as output.
}

void transmitter_sendMeasurement(const char *prefix, int16_t val)
{
    char b[11]; // Minus symbol + 5 max digits + asteriks + max 3 digits + terminator.

    // Measurement to string.
    toString(val, b);

    uint8_t i = 0;

    // Get checksum.
    uint8_t checksum = 0;
    while (b[i] != '\0')
    {
        checksum ^= b[i++];
    }
    b[i++] = '*';

    // Checksum to string in the same buffer as measurement w/ offset.
    toString(checksum, b + i);

    // Write prefix.
    i = 0;
    while (prefix[i] != '\0')
    {
        writeByte(prefix[i++]);
    }

    // Write separator.
    writeByte(':');

    // Write value w/ checksum.
    i = 0;
    while (b[i] != '\0')
    {
        writeByte(b[i++]);
    }

    writeByte('\n');
}

namespace
{
    void toString(int16_t val, char *buf)
    {

        uint16_t divider = 1;
        uint8_t idxCntr = 0;

        // Check negative value.
        if (val < 0)
        {
            buf[idxCntr++] = '-';
            val *= -1;
        }

        int16_t valTmp = val;

        // Get max divider to get num of digits.
        while (valTmp >= 10)
        {
            divider *= 10;
            valTmp /= 10;
        }

        // Convert value starting from highest digit
        // i.e. "1" ... to "3" in 1223.
        while (divider)
        {
            uint8_t digitInt = val / divider;
            val -= digitInt * divider;
            divider /= 10;

            buf[idxCntr++] = digitInt + '0'; // To ASCII.
        }

        buf[idxCntr] = '\0';
    }

    void writeByte(char b)
    {
        constexpr uint32_t baud = 9600;
        constexpr uint16_t delay = (F_CPU / baud) / 4;
        constexpr uint16_t sub = 15 / 4;
        constexpr uint16_t txBitDelay = (delay > sub) ? delay - sub : 1; //!< Fine tuned delay for soft serial.

        uint8_t mask = (1 << serial);
        uint8_t invMask = ~mask;

        cli();

        // Start bit.
        PORTB &= invMask;
        _delay_loop_2(txBitDelay);

        // Byte bits.
        for (uint8_t i = 0; i < 8; i++)
        {
            if (b & 1)
                PORTB |= mask; // 1
            else
                PORTB &= invMask; // 0

            _delay_loop_2(txBitDelay);
            b >>= 1;
        }

        // Restore state.
        PORTB |= mask;
        _delay_loop_2(txBitDelay);

        sei();
    }
}