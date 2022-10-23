#include "controllers.hpp"
#include <zephyr/logging/log.h>
#include "config/software.hpp"

LOG_MODULE_REGISTER(controllers, CONTROLLERS_LOG_LEVEL);

namespace app
{
    namespace io
    {
        namespace controllers
        {

            ClosedController::ClosedController(const device &actuator, uint32_t actuatorChannel,
                                               const device &sensor, uint8_t sensorChannel)
                : m_actuator(actuator, actuatorChannel), m_sensor(sensor, sensorChannel),
                  filter(0.19) {}

            void ClosedController::updateSelf()
            {
                uint8_t state = m_sensor.getState();
                state = app::helpers::exponential(255 - state, 0, 255);
                m_valSelf = filter.read(state);
            }

            bool ClosedController::init()
            {
                if (!m_sensor.init())
                {
                    LOG_ERR("Failed to initialize controller's sensor");
                    return false;
                }
                if (!m_actuator.init())
                {
                    LOG_ERR("Failed to initialize controller's actuator");
                    return false;
                }
                return true;
            };

            void ClosedController::setControlStrategy(ControlStrategy strategy)
            {
                m_currStrategy = strategy;
                update();
            }

            ClosedController::ControlStrategy ClosedController::getControlStrategy()
            {
                return m_currStrategy;
            }

            void ClosedController::setState(uint8_t state)
            {
                m_valZigbee = state;
                update();
            }

            void ClosedController::update()
            {
                if (m_currStrategy == ControlStrategy::SELF)
                {
                    updateSelf();
                    m_actuator.setState(m_valSelf);
                }
                else
                {
                    filter.reset();
                    m_actuator.setState(m_valZigbee);
                }
            }

            bool ClosedController::ok() const
            {
                return (m_sensor.ok() && m_actuator.ok());
            }

            uint8_t ClosedController::readSensor()
            {
                return m_sensor.getState();
            }
        }
    }
}