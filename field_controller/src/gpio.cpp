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
            Output::Output(const gpio_dt_spec &pin, const char *label)
                : m_pin(pin), m_label(label) {}

            bool Output::init()
            {
                LOG_INF("Initializing %s", m_label);
                m_err = !device_is_ready(m_pin.port);
                if (m_err)
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
                m_err = gpio_pin_set_dt(&m_pin, state);
                if (m_err)
                {
                    LOG_ERR("%s failed to set state to %d", m_label, state);
                }
            }

            bool Output::ok() const
            {
                return !m_err;
            }
        }

        namespace analog
        {
            PWM::PWM(const device &pwm, uint32_t channel)
                : m_pwm(&pwm), m_channel(channel) {}

            bool PWM::init()
            {
                m_err = !device_is_ready(m_pwm);
                if (m_err)
                {
                    LOG_ERR("PWM device is not ready");
                    return false;
                }

                LOG_DBG("Calibrating PWM device");

                return true;
            }

            void PWM::setState(uint8_t state)
            {
                m_err = pwm_set(m_pwm, m_channel, PWM_USEC(255), PWM_USEC(state), PWM_POLARITY_NORMAL);
                // m_err = led_set_channel(m_pwm, m_channel, state);
                if (m_err)
                {
                    LOG_ERR("Failed to set PWM");
                }
            }

            bool PWM::ok() const
            {
                return !m_err;
            }

            ADC::ADC(const device &adc, uint8_t channel) : m_adc(&adc)
            {
                m_cfg.channel_id = channel;

#ifdef CONFIG_ADC_NRFX_SAADC
#define ADC_INPUT_POS_OFFSET SAADC_CH_PSELP_PSELP_AnalogInput0
#else
#define ADC_INPUT_POS_OFFSET 0
#endif

#ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
                m_cfg.input_positive = ADC_INPUT_POS_OFFSET + channel;
#endif
            }

            bool ADC::init()
            {
                m_err = !device_is_ready(m_adc);
                if (m_err)
                {
                    LOG_ERR("ADC device is not ready");
                    return false;
                }

                adc_channel_setup(m_adc, &m_cfg);
                if (m_err)
                {
                    LOG_ERR("Failed to setup ADC channel");
                    return false;
                }
                m_seq.channels = BIT(m_cfg.channel_id);
                return true;
            }

            uint8_t ADC::getState()
            {
                m_err = adc_read(m_adc, &m_seq);
                if (m_err)
                {
                    LOG_ERR("ADC reading failed");
                    return 0;
                }
                return m_buf / LED_ADC_DIVIDER_TO_U8;
            }

            bool ADC::ok() const
            {
                return !m_err;
            }
        }
    }
}