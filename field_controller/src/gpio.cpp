#include "gpio.hpp"

namespace app
{
    namespace io
    {
        namespace digital
        {
            bool Output::init(const gpio_dt_spec &pin)
            {
                m_pin = pin;
                if (!device_is_ready(m_pin.port))
                {
                    return false;
                }

                if (gpio_pin_configure_dt(&m_pin, GPIO_OUTPUT_INACTIVE) < 0)
                {
                    return false;
                }

                return true;
            }

            void Output::setState(bool state)
            {
                m_err = (gpio_pin_set_dt(&m_pin, state) < 0);
            }

            bool Output::ok() const { return !m_err; }
        }
    }
}