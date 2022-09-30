#pragma once
#include <cstdint>
#include <drivers/gpio.h>

namespace app
{
    namespace io
    {
        namespace digital
        {
            class Output
            {
            private:
                gpio_dt_spec m_pin;
                bool m_err = false;
            public:
                bool init(const gpio_dt_spec &pin);
                void setState(bool state);
                bool ok() const;
            };

            class Input
            {
            public:
                bool init() { return true; }
                bool getState() { return true; }
                bool ok() { return true; }
            };
        }

        namespace analog
        {
            class PWM
            {
            public:
                bool init() { return true; }
                void setState(uint16_t state) {}
                bool ok() { return true; }
            };

            class ADC
            {
            public:
                bool init() { return true; }
                uint8_t getState() { return 0; }
                bool ok() { return true; }
            };
        }
    }
}