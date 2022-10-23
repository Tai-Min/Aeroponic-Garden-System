#pragma once
#include "gpio.hpp"
#include "helpers.hpp"
#include "types/controllers_types.hpp"

namespace app
{
    namespace io
    {
        namespace controllers
        {

            class ClosedController
            {
            public:
                using ControlStrategy = ClosedControlStrategy::Value;

            private:
                using AnIn = app::io::analog::ADC;
                using AnOut = app::io::analog::PWM;
                using LowPassFilter = app::helpers::LowPassFilter;

                AnIn m_sensor;
                AnOut m_actuator;
                LowPassFilter filter;
                ControlStrategy m_currStrategy = ControlStrategy::SELF;
                uint8_t m_selfThreshold = 0;
                uint8_t m_valZigbee = 0;
                uint8_t m_valSelf = 128;

                void updateSelf();

            public:
                ClosedController(const device &actuator, uint32_t actuatorChannel,
                                 const device &sensor, uint8_t sensorChannel);
                bool init();
                void update();
                bool ok() const;

                void setControlStrategy(ControlStrategy strategy);
                ControlStrategy getControlStrategy();

                void setState(uint8_t state);
                uint8_t readSensor();
            };
        }
    }
}