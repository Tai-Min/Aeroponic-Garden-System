#pragma once
#include <cstdint>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <drivers/adc.h>
#include "config/hardware.hpp"

namespace app
{
    namespace io
    {
        namespace digital
        {
            class Output
            {
            private:
                const gpio_dt_spec &m_pin;
                bool m_err = false;
                const char *m_label;

            public:
                Output(const gpio_dt_spec &pin, const char *label);
                bool init();
                void setState(bool state);
                bool ok() const;
                constexpr const char *getLabel() const { return m_label; };
            };
        }

        namespace analog
        {
            class PWM
            {
            private:
                const device *m_pwm;
                const uint32_t m_channel;
                bool m_err = false;

            public:
                PWM(const device &pwm, uint32_t channel);
                bool init();
                void setState(uint8_t state);
                bool ok() const;
            };

            class ADC
            {
            private:
                uint16_t m_buf;
                adc_channel_cfg m_cfg = {
                    .gain = LED_ADC_GAIN,
                    .reference = LED_ADC_REFERENCE,
                    .acquisition_time = LED_ADC_ACQUISITION_TIME,
                    .channel_id = 0,
                    .differential = 0};
                adc_sequence m_seq = {
                    .channels = 0,
                    .buffer = &m_buf,
                    .buffer_size = sizeof(uint16_t),
                    .resolution = LED_ADC_RESOLUTION};

                const device *m_adc;
                bool m_err = false;

            public:
                ADC(const device &adc, uint8_t channelId);
                bool init();
                uint8_t getState();
                bool ok() const;
            };
        }
    }
}