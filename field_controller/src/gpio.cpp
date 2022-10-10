#include "gpio.hpp"
#include <zephyr/logging/log.h>
#include "config/software.hpp"

LOG_MODULE_REGISTER(gpio, GPIO_LOG_LEVEL);

namespace app
{
    namespace io
    {
        namespace digital
        {
            Output::Output(const char *label) : m_label(label) {}

            bool Output::init(const gpio_dt_spec &pin)
            {
                LOG_INF("Initializing %s", m_label);
                m_pin = pin;
                if (!device_is_ready(m_pin.port))
                {
                    LOG_ERR("%s is not ready", m_label);
                    return false;
                }

                if (gpio_pin_configure_dt(&m_pin, GPIO_OUTPUT_INACTIVE) < 0)
                {
                    LOG_ERR("Couldn't configure %s as GPIO_OUTPUT_INACTIVE", m_label);
                    return false;
                }
                LOG_INF("%s initialized", m_label);
                return true;
            }

            void Output::setState(bool state)
            {
                LOG_DBG("Setting %s to %d", m_label, state);
                m_err = (gpio_pin_set_dt(&m_pin, state) < 0);
                if (m_err)
                {
                    LOG_ERR("%s failed to set state to %d", m_label, state);
                }
            }

            bool Output::ok() const
            {
                return !m_err;
            }

            const char *Output::getLabel()
            {
                return m_label;
            }
        }
    }
}