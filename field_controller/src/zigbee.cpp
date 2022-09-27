#include "zigbee/zboss_api_hell_inputs.hpp"

namespace app
{
    namespace networking
    {
        namespace zigbee
        {
            bool ZigbeeInterface::m_isInit = false;
            ZigbeeInterface::DeviceContext ZigbeeInterface::m_devCtx;
            ZigbeeInterface ZigbeeInterface::m_instance = ZigbeeInterface();
            Callback ZigbeeInterface::m_callbacks[CallbackType::length()] = {nullptr};

            void ZigbeeInterface::btnCallback(uint32_t button_state, uint32_t has_changed)
            {
            }

            void ZigbeeInterface::zclDeviceCallback(zb_bufid_t bufid)
            {
            }

            void ZigbeeInterface::toggleIdentifyDiode(zb_bufid_t bufid)
            {
                // static int blink = 0;
                //  TODO: toggle identify led
                ZB_SCHEDULE_APP_ALARM(toggleIdentifyDiode, bufid, ZB_MILLISECONDS_TO_BEACON_INTERVAL(500));
            }

            void ZigbeeInterface::identifyCallback(zb_bufid_t bufid)
            {
                if (bufid)
                {
                    ZB_SCHEDULE_APP_CALLBACK(toggleIdentifyDiode, bufid);
                }
                else
                {
                    ZB_SCHEDULE_APP_ALARM_CANCEL(toggleIdentifyDiode, ZB_ALARM_ANY_PARAM);
                    // TODO: off identify led.
                }
            }

            void ZigbeeInterface::clusterAttrInit()
            {
                m_devCtx.basic.zcl_version = ZB_ZCL_VERSION;
                m_devCtx.basic.app_version = BASIC_APP_VER;
                m_devCtx.basic.stack_version = BASIC_ZB_STACK_VER;
                m_devCtx.basic.hw_version = BASIC_HW_VER;

                ZB_ZCL_SET_STRING_VAL(m_devCtx.basic.mf_name,
                                      BASIC_MANUF_NAME,
                                      ZB_ZCL_STRING_CONST_SIZE(BASIC_MANUF_NAME));

                ZB_ZCL_SET_STRING_VAL(m_devCtx.basic.model_id,
                                      BASIC_MODEL_ID,
                                      ZB_ZCL_STRING_CONST_SIZE(BASIC_MODEL_ID));

                ZB_ZCL_SET_STRING_VAL(m_devCtx.basic.date_code,
                                      BASIC_DATE_CODE,
                                      ZB_ZCL_STRING_CONST_SIZE(BASIC_DATE_CODE));

                m_devCtx.basic.power_source = BASIC_POWER_SOURCE;

                ZB_ZCL_SET_STRING_VAL(
                    m_devCtx.basic.location_id,
                    BASIC_LOCATION,
                    ZB_ZCL_STRING_CONST_SIZE(BASIC_LOCATION));

                m_devCtx.basic.ph_env = BASIC_ENV;

                m_devCtx.identify.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
            }

            bool ZigbeeInterface::init() const
            {
                if (m_isInit)
                {
                    LOG_WRN("Init called more than once, skipping");
                    return true;
                }

                int err = dk_buttons_init(btnCallback);
                if (err)
                {
                    LOG_ERR("DK buttons init failed");
                    return false;
                }

                err = dk_leds_init();
                if (err)
                {
                    LOG_ERR("DK LEDs init failed");
                    return false;
                }

                register_factory_reset_button(FACTORY_RESET_BUTTON);

                ZB_ZCL_REGISTER_DEVICE_CB(zclDeviceCallback);
                ZB_AF_REGISTER_DEVICE_CTX(&ctrlCtx);

                clusterAttrInit();

                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_FAN_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_LED1_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_LED2_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_STRATEGY_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_WATER_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_NUTRI_ENDPOINT, identifyCallback);

                zcl_scenes_init();

                m_isInit = true;
                LOG_INF("Initialized");
                return true;
            }

            void ZigbeeInterface::doWorkDetached() const
            {
                zigbee_enable();
                LOG_INF("Worker started");
            }
        }
    }
}