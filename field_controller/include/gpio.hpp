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
                bool init();
                void setState(bool state);
                bool ok();
            };

            class Input
            {
            public:
                bool init();
                bool getState();
                bool ok();
            };
        }

        namespace analog
        {
            class PWM
            {
            public:
                bool init();
                void setState(uint16_t state);
                bool ok();
            };

            class ADC
            {
            public:
                bool init();
                uint8_t getState();
                bool ok();
            };
        }
    }
}