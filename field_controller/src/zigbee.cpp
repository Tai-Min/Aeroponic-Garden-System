#include "zigbee.hpp"
#include <dk_buttons_and_leds.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>

extern "C"
{
#include <zb_nrf_platform.h>
}

#include "zigbee/defines.hpp"

LOG_MODULE_REGISTER(zigbee, LOG_LEVEL_DBG);

namespace
{
    using namespace app::networking::zigbee;

    ZigbeeInterface iface = ZigbeeInterface::getInterface();

    ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(
        basicAttrList,
        &iface.getContext().basic.zcl_version,
        &iface.getContext().basic.app_version,
        &iface.getContext().basic.stack_version,
        &iface.getContext().basic.hw_version,
        iface.getContext().basic.mf_name,
        iface.getContext().basic.model_id,
        iface.getContext().basic.date_code,
        &iface.getContext().basic.power_source,
        iface.getContext().basic.location_id,
        &iface.getContext().basic.ph_env,
        iface.getContext().basic.sw_ver);

    ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
        identifyAttrList,
        &iface.getContext().identify.identify_time);

    ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(
        groupsAttrList,
        &iface.getContext().groups.name_support);

    ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(
        scenesAttrList,
        &iface.getContext().scenes.scene_count,
        &iface.getContext().scenes.current_scene,
        &iface.getContext().scenes.current_group,
        &iface.getContext().scenes.scene_valid,
        &iface.getContext().scenes.name_support);

    // Fan
    ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
        fanAttrList,
        &iface.getContext().fanAttr.on_off);

    ZB_DECLARE_ON_OFF_CLUSTER_LIST(
        fanClusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        fanAttrList);

    ZB_DECLARE_EP(
        fanEp,
        ZB_FAN_ENDPOINT,
        fanClusters);

    // LED 1
    ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST(
        led1AttrList,
        &iface.getContext().led1Attr.current_level,
        &iface.getContext().led1Attr.remaining_time);

    ZB_DECLARE_LEVEL_CLUSTER_LIST(
        led1Clusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        led1AttrList);

    ZB_DECLARE_EP(
        led1Ep,
        ZB_LED1_ENDPOINT,
        led1Clusters);

    // LED 2
    ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST(
        led2AttrList,
        &iface.getContext().led2Attr.current_level,
        &iface.getContext().led2Attr.remaining_time);

    ZB_DECLARE_LEVEL_CLUSTER_LIST(
        led2Clusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        led2AttrList);

    ZB_DECLARE_EP(
        led2Ep,
        ZB_LED2_ENDPOINT,
        led2Clusters);

    // LED strategy
    ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
        ledStrategyAttrList,
        &iface.getContext().ledStrategyAttr.on_off);

    ZB_DECLARE_ON_OFF_CLUSTER_LIST(
        ledStrategyClusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        ledStrategyAttrList);

    ZB_DECLARE_EP(
        ledStrategyEp,
        ZB_STRATEGY_ENDPOINT,
        ledStrategyClusters);

    // Relay water.
    ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
        waterAttrList,
        &iface.getContext().waterAttr.on_off);

    ZB_DECLARE_ON_OFF_CLUSTER_LIST(
        waterClusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        waterAttrList);

    ZB_DECLARE_EP(
        relayWaterEp,
        ZB_WATER_ENDPOINT,
        waterClusters);

    // Relay nutri.
    ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
        nutriAttrList,
        &iface.getContext().nutriAttr.on_off);

    ZB_DECLARE_ON_OFF_CLUSTER_LIST(
        nutriClusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        nutriAttrList);

    ZB_DECLARE_EP(
        relayNutriEp,
        ZB_NUTRI_ENDPOINT,
        nutriClusters);

    // Endpoint declatarion.
    ZBOSS_DECLARE_DEVICE_CTX_6_EP(
        ctrlCtx,
        fanEp,
        led1Ep,
        led2Ep,
        ledStrategyEp,
        relayWaterEp,
        relayNutriEp);
}

extern "C"
{
    void zboss_signal_handler(zb_bufid_t bufid)
    {
        zigbee_led_status_update(bufid, ZIGBEE_NETWORK_STATE_LED);

        ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

        if (bufid)
        {
            zb_buf_free(bufid);
        }
    }
}

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
                //static int blink = 0;
                // TODO: toggle identify led
                ZB_SCHEDULE_APP_ALARM(toggleIdentifyDiode, bufid, ZB_MILLISECONDS_TO_BEACON_INTERVAL(100));
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

                int err = dk_buttons_init(ZigbeeInterface::btnCallback);
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

                ZB_ZCL_REGISTER_DEVICE_CB(ZigbeeInterface::zclDeviceCallback);
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