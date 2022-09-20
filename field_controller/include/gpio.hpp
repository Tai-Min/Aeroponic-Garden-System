#pragma once
#include <cstdint>

namespace app
{
    namespace io
    {
        namespace digital
        {
            class Output
            {
            public:
                bool init() { return true; }
                void setState(bool state) {}
                bool ok() { return true; }
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