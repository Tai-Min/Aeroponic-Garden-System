#pragma once

#include "types/zigbee_types.hpp"
#include "gpio.hpp"

namespace app
{
    namespace networking
    {
        namespace zigbee
        {
            class ZigbeeInterface
            {
                friend void ::zboss_signal_handler(zb_bufid_t bufid);

            private:
                static DeviceContext m_devCtx;

                static bool m_isInit;
                //!< Singleton pattern. There can be only one Zigbee interface on the device.
                static ZigbeeInterface m_instance;

                static app::io::digital::Output m_identifyLed;

                //!< Callbacks to be called on msg rx.
                static Callback m_callbacks[CallbackType::length()];

                static MeasurementCallback m_sensorCallback;

                static void startIdentify(zb_bufid_t bufid);
                static void btnCallback(uint32_t button_state, uint32_t has_changed);
                static void sensorReadCallback(zb_bufid_t bufid);
                static void setTemperature(int16_t temp);
                static void setPressure(int16_t press);
                static void setHumidity(int16_t hum);
                static void zclDeviceCallback(zb_bufid_t bufid);
                static void toggleIdentify(zb_bufid_t bufid);
                static void identifyCallback(zb_bufid_t bufid);
                static void clusterAttrInit();

            public:
                /**
                 * @brief Get instance of Zigbee hardware interface.
                 *
                 * @return Reference to instance of hardware interface.
                 */
                constexpr static ZigbeeInterface &getInterface()
                {
                    return m_instance;
                }

                /**
                 * @brief Initialize hardware interface.
                 *
                 * @return True on success.
                 */
                bool init() const;

                /**
                 * @brief Register callback to be called on specific message receive.
                 *
                 * @param type Type of callback.
                 * @param callback Function to call.
                 */
                void registerCallback(CallbackType::Values type, Callback callback)
                {
                    m_callbacks[static_cast<uint8_t>(type)] = callback;
                };

                void registerSensorCallback(MeasurementCallback cb)
                {
                    m_sensorCallback = cb;
                }

                /**
                 * @brief Run Zigbee worker. Call after init().
                 */
                void doWorkDetached() const;

                bool ok() const { return true; }

                constexpr const DeviceContext &getContext() { return m_devCtx; }
            };
        }
    }
}