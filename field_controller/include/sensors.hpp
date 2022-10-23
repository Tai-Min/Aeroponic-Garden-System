#pragma once
#include <cstdint>
#include <drivers/sensor.h>

namespace app
{
    namespace sensors
    {
        namespace environmental
        {
            class BME688
            {
            private:
                bool m_err = false;
                const device *m_dev;

            public:
                BME688(const device *d);
                bool init();
                bool read();
                bool ok() const;

                float getTemperature();
                float getPressure();
                float getHumidity();
                float getGasResistance();
            };
        }
    }
}