#include "timer.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

void timer_setup();
void timer_reset();
uint32_t timer_read();

ISR(TIMER1_COMPA_vect);

//****************
// Implementation.
//****************

namespace
{
    volatile uint32_t timerReg = 0;
    bool isSetup = false;
}

void timer_setup()
{
    if (isSetup)
        return;

    cli();
    uint16_t microsecondMatchOvf = F_CPU / 100000;
    TCCR1B |= (1 << WGM12) | (1 << CS10);

    OCR1AH = (microsecondMatchOvf >> 8);
    OCR1AL = microsecondMatchOvf;

    TIMSK1 |= (1 << OCIE1A);
    sei();
}

void timer_reset()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        timerReg = 0;
    }
}

uint32_t timer_read()
{
    uint32_t r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        r = timerReg;
    }
    return r;
}

ISR(TIMER1_COMPA_vect)
{
    timerReg += 10;
}