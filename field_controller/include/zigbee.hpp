#pragma once
#include "types/zigbee_types.hpp"

namespace app
{
    namespace networking
    {
        namespace zigbee
        {
            class ZigbeeInterface
            {
            private:
                //!< Singleton pattern. There can be only one Zigbee interface on the device.
                static ZigbeeInterface m_instance;

                //!< Callbacks to be called on msg rx.
                Callback m_callbacks[CallbackType::length()] = {nullptr};

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

                /**
                 * @brief Send feedback message via Zigbee connection.
                 *
                 * @param type Type of feedback.
                 * @param value Value.
                 */
                void sendFeedbackAsync(FeedbackType::Values type, uint16_t value) const;

                /**
                 * @brief Run Zigbee worker. Call after init().
                 */
                void doWorkDetached() const;

                bool ok() const;
            };
        }
    }
}