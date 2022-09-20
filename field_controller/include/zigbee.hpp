#pragma once
#include "types/zigbee_types.hpp"

#define _Static_assert static_assert

extern "C"
{
#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zigbee/zigbee_zcl_scenes.h>
}

extern "C" void zboss_signal_handler(zb_bufid_t bufid);

namespace app
{
    namespace networking
    {
        namespace zigbee
        {
            class ZigbeeInterface
            {
            private:
                struct DeviceContext
                {
                    zb_zcl_basic_attrs_ext_t basic;
                    zb_zcl_identify_attrs_t identify;
                    zb_zcl_scenes_attrs_t scenes;
                    zb_zcl_groups_attrs_t groups;
                    zb_zcl_on_off_attrs_t fanAttr;
                    zb_zcl_level_control_attrs_t led1Attr;
                    zb_zcl_level_control_attrs_t led2Attr;
                    zb_zcl_on_off_attrs_t ledStrategyAttr;
                    zb_zcl_on_off_attrs_t waterAttr;
                    zb_zcl_on_off_attrs_t nutriAttr;
                };

                static DeviceContext m_devCtx;

                static bool m_isInit;
                //!< Singleton pattern. There can be only one Zigbee interface on the device.
                static ZigbeeInterface m_instance;

                //!< Callbacks to be called on msg rx.
                static Callback m_callbacks[CallbackType::length()];

                static void btnCallback(uint32_t button_state, uint32_t has_changed);
                static void zclDeviceCallback(zb_bufid_t bufid);
                static void toggleIdentifyDiode(zb_bufid_t bufid);
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

                /**
                 * @brief Send feedback message via Zigbee connection.
                 *
                 * @param type Type of feedback.
                 * @param value Value.
                 */
                void sendFeedbackAsync(FeedbackType::Values type, uint16_t value) const {}

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