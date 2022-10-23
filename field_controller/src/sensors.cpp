#include "sensors.hpp"
#include <zephyr/logging/log.h>

#include "config/hardware.hpp"
#include "config/software.hpp"

LOG_MODULE_REGISTER(sensors, SENSORS_LOG_LEVEL);

namespace app
{
    namespace sensors
    {
        namespace environmental
        {

            BME688::BME688(const device *d) : m_dev(d) {}

            bool BME688::init()
            {
                LOG_INF("BME688 initializing");
                m_err = !device_is_ready(m_dev);
                if (m_err)
                {
                    LOG_ERR("Failed to get environmental Sensor device struct");
                    return false;
                }
                LOG_INF("BME688 initialized");
                return true;
            }

            bool BME688::read()
            {
                m_err = sensor_sample_fetch(m_dev);
                if (m_err)
                {
                    LOG_ERR("Failed to fetch environmental sensor");
                }
                return !m_err;
            }

            bool BME688::ok() const
            {
                return !m_err;
            }

            float BME688::getTemperature()
            {
                sensor_value val;
                m_err = sensor_channel_get(m_dev, SENSOR_CHAN_AMBIENT_TEMP, &val);
                if (m_err)
                {
                    LOG_ERR("Failed to get temperature");
                    return 0;
                }
                return val.val1 + val.val2 / 100000.0;
            }

            float BME688::getPressure()
            {
                sensor_value val;
                m_err = sensor_channel_get(m_dev, SENSOR_CHAN_PRESS, &val);
                if (m_err)
                {
                    LOG_ERR("Failed to get pressure");
                    return 0;
                }
                return val.val1 + val.val2 / 100000.0;
            }

            float BME688::getHumidity()
            {
                sensor_value val;
                m_err = sensor_channel_get(m_dev, SENSOR_CHAN_HUMIDITY, &val);
                if (m_err)
                {
                    LOG_ERR("Failed to get humidity");
                    return 0;
                }
                return val.val1 + val.val2 / 100000.0;
            }

            float BME688::getGasResistance()
            {
                sensor_value val;
                m_err = sensor_channel_get(m_dev, SENSOR_CHAN_GAS_RES, &val);
                if (m_err)
                {
                    LOG_ERR("Failed to get gas resistance");
                    return 0;
                }
                return val.val1 + val.val2 / 100000.0;
            }
        }
    }
}