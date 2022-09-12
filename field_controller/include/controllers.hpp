#pragma once
#include "gpio.hpp"
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

                AnIn m_sensor;
                AnOut m_actuator;
                ControlStrategy m_currStrategy = ControlStrategy::SELF;
                uint16_t m_selfThreshold = 0;
                uint16_t m_valZigbee = 0;
                uint16_t m_valSelf = 0;

                void updateSelf()
                {
                }

            public:
                bool init(){
                    return true;
                };

                void setControlStrategy(ControlStrategy strategy)
                {
                    m_currStrategy = strategy;
                    update();
                }

                void setState(uint16_t state)
                {
                    m_valZigbee = state;
                    update();
                }

                void update()
                {
                    if (m_currStrategy == ControlStrategy::SELF)
                    {
                        updateSelf();
                        m_actuator.setState(m_valSelf);
                    }
                    else
                    {
                        m_actuator.setState(m_valZigbee);
                    }
                }

                bool ok() const
                {
                    return true;
                }

                uint16_t readSensor(){
                    return 0;
                }
            };
        }
    }
}