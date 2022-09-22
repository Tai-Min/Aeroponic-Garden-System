#ifdef BLINK_TEST
#include "timer.hpp"
#else
#include <util/delay.h>
#include "transmitter.hpp"
#include "level_sensor.hpp"
#include "ph_sensor.hpp"
#include "tds_sensor.hpp"
#include "hw_definitions.hpp"
#endif

#ifndef BLINK_TEST
int main()
{
  _delay_ms(30000);
  transmitter_setup();
  usensor_commonSetup();
  phsensor_setup();
  tdssensor_setup();

  while (true)
  {
    transmitter_sendMeasurement(u0Prefix, usensor_read(u0));
    transmitter_sendMeasurement(u1Prefix, usensor_read(u1));
    transmitter_sendMeasurement(phPrefix, phsensor_read());
    transmitter_sendMeasurement(ecPrefix, tdssensor_read());
    _delay_ms(10);
  }
}
#else
int main()
{
  timer_setup();
  DDRB |= (1 << DDB0);

  while (true)
  {
    PORTB |= (1 << PB0);
    timer_reset();
    while (timer_read() < 1000000L)
      ;
    PORTB &= ~(1 << PB0);
    timer_reset();
    while (timer_read() < 1000000L)
      ;
  }
}
#endif