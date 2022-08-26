#include <util/delay.h>
#include <avr/io.h>
#include "transmitter.hpp"
#include "level_sensor.hpp"
#include "ph_sensor.hpp"
#include "ec_sensor.hpp"

namespace
{
  constexpr char u0Prefix[] = "u0";
  constexpr LevelSensorHardware u0 = {
      .reg = &DDRB,
      .regBit = DDB0,
      .port = &PORTB,
      .portBit = PORTB0,
      .pin = &PINB,
      .pinBit = PINB0};

  constexpr char u1Prefix[] = "u1";
  constexpr LevelSensorHardware u1 = {
      .reg = &DDRB,
      .regBit = DDB0,
      .port = &PORTB,
      .portBit = PORTB0,
      .pin = &PINB,
      .pinBit = PINB0};

  constexpr char phPrefix[] = "ph";
  constexpr char ecPrefix[] = "ec";
};

int main()
{
  transmitter_setup();
  usensor_setup();
  phsensor_setup();
  ecsensor_setup();

  while (true)
  {
    transmitter_sendMeasurement(u0Prefix, usensor_read(u0));
    transmitter_sendMeasurement(u1Prefix, usensor_read(u1));
    transmitter_sendMeasurement(phPrefix, phsensor_read());
    transmitter_sendMeasurement(ecPrefix, ecsensor_read());
    _delay_ms(1);
  }
}