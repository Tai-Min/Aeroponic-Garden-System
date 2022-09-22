#pragma once
#include <stdint.h>
#include <avr/io.h>

constexpr uint8_t serial = PB0;

constexpr char u0Prefix[] = "u0";
constexpr uint8_t u0 = PB1;

constexpr char u1Prefix[] = "u1";
constexpr uint8_t u1 = PB2;

constexpr char phPrefix[] = "ph";
constexpr uint8_t ph = PB3;

constexpr char ecPrefix[] = "tds";
constexpr uint8_t ec = PB4;