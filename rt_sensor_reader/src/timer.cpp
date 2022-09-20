#include "timer.hpp"
#include <avr/interrupt.h>
#include <util/atomic.h>

void timer_setup();
void timer_reset();
uint32_t timer_read();

/**
 * @brief Timer's interrupt handler.
 */
ISR(TIMER1_COMPA_vect);

//****************
// Implementation.
//****************

namespace
{
    volatile uint32_t timerReg = 0; //!< Holds timer's value in us.
}

void timer_setup()
{
    cli();
    uint8_t microsecondMatchOvf = F_CPU / 100000; // In reality this timer is working in increments of 10us
    TCCR0A |= (1 << WGM01);                       // CTC
    TCCR0B |= (1 << CS00);                        // /1 prescaler

    OCR0A = microsecondMatchOvf;

    TIMSK |= (1 << OCIE0A); // Enable TIMER0_COMPA_vect
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

ISR(TIMER0_COMPA_vect)
{
    timerReg += 10;
}