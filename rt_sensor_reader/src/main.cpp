#include <avr/io.h>

#ifdef BLINK_TEST
#include "timer.hpp"
#else
#include <util/delay.h>
#include "transmitter.hpp"
#include "level_sensor.hpp"
#include "ph_sensor.hpp"
#include "ec_sensor.hpp"
#endif

#ifndef BLINK_TEST
namespace
{
#ifdef UNO_DEBUG
  constexpr SerialHardware serial = {};
#else
  constexpr SerialHardware serial = {
      .reg = &DDRB,
      .regBit = DDB0,
      .port = &PORTB,
      .portBit = PORTB0};
#endif

  constexpr char u0Prefix[] = "u0";
#ifdef UNO_DEBUG
  // UNO pin 8
  constexpr LevelSensorHardware u0 = {
      .reg = &DDRB,
      .regBit = DDB0,
      .port = &PORTB,
      .portBit = PORTB0,
      .pin = &PINB,
      .pinBit = PINB0};
#else
  constexpr LevelSensorHardware u0 = {
      .reg = &DDRB,
      .regBit = DDB1,
      .port = &PORTB,
      .portBit = PORTB1,
      .pin = &PINB,
      .pinBit = PINB1};
#endif

  constexpr char u1Prefix[] = "u1";
#ifdef UNO_DEBUG
  // UNO pin 9
  constexpr LevelSensorHardware u1 = {
      .reg = &DDRB,
      .regBit = DDB1,
      .port = &PORTB,
      .portBit = PORTB1,
      .pin = &PINB,
      .pinBit = PINB1};
#else
  constexpr LevelSensorHardware u1 = {
      .reg = &DDRB,
      .regBit = DDB2,
      .port = &PORTB,
      .portBit = PORTB2,
      .pin = &PINB,
      .pinBit = PINB2};
#endif
  constexpr char phPrefix[] = "ph";
  constexpr char ecPrefix[] = "ec";
};

int main()
{
  transmitter_setup(serial);
  usensor_common_setup();
  phsensor_setup();
  ecsensor_setup();

  while (true)
  {
    transmitter_sendMeasurement(u0Prefix, usensor_read(u0));
    transmitter_sendMeasurement(u1Prefix, usensor_read(u1));
    transmitter_sendMeasurement(phPrefix, phsensor_read());
    transmitter_sendMeasurement(ecPrefix, ecsensor_read());
    _delay_ms(50);
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