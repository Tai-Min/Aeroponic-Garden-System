#pragma once

#include "zigbee.hpp"
#include <dk_buttons_and_leds.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>

extern "C"
{
#include <zboss_api_addons.h>
#include <zigbee/zigbee_zcl_scenes.h>
#include <zb_nrf_platform.h>
}

#include "config/hardware.hpp"
#include "config/software.hpp"
#include "config/zigbee.hpp"

LOG_MODULE_REGISTER(zigbee, ZIGBEE_LOG_LEVEL);

namespace
{
#undef ZB_ZCL_START_PACKET_REQ
#undef ZB_ZCL_CONSTRUCT_COMMAND_HEADER_REQ

#define ZB_ZCL_START_PACKET_REQ(zbbuf)  static_cast<zb_uint8_t*>(zb_zcl_start_command_header((zbbuf),
#define ZB_ZCL_CONSTRUCT_COMMAND_HEADER_REQ(data_ptr, tsn, cmd_id) (cmd_id), NULL))

#define ZB_DECLARE_BOOL_CLUSTER_LIST(                                 \
    cluster_list_name,                                                \
    basic_attr_list,                                                  \
    identify_attr_list,                                               \
    groups_attr_list,                                                 \
    scenes_attr_list,                                                 \
    on_off_attr_list)                                                 \
    zb_zcl_cluster_desc_t cluster_list_name[] =                       \
        {                                                             \
            ZB_ZCL_CLUSTER_DESC(                                      \
                ZB_ZCL_CLUSTER_ID_IDENTIFY,                           \
                ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t), \
                (identify_attr_list),                                 \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                           \
                ZB_ZCL_MANUF_CODE_INVALID),                           \
            ZB_ZCL_CLUSTER_DESC(                                      \
                ZB_ZCL_CLUSTER_ID_BASIC,                              \
                ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),    \
                (basic_attr_list),                                    \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                           \
                ZB_ZCL_MANUF_CODE_INVALID),                           \
            ZB_ZCL_CLUSTER_DESC(                                      \
                ZB_ZCL_CLUSTER_ID_SCENES,                             \
                ZB_ZCL_ARRAY_SIZE(scenes_attr_list, zb_zcl_attr_t),   \
                (scenes_attr_list),                                   \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                           \
                ZB_ZCL_MANUF_CODE_INVALID),                           \
            ZB_ZCL_CLUSTER_DESC(                                      \
                ZB_ZCL_CLUSTER_ID_GROUPS,                             \
                ZB_ZCL_ARRAY_SIZE(groups_attr_list, zb_zcl_attr_t),   \
                (groups_attr_list),                                   \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                           \
                ZB_ZCL_MANUF_CODE_INVALID),                           \
            ZB_ZCL_CLUSTER_DESC(                                      \
                ZB_ZCL_CLUSTER_ID_ON_OFF,                             \
                ZB_ZCL_ARRAY_SIZE(on_off_attr_list, zb_zcl_attr_t),   \
                (on_off_attr_list),                                   \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                           \
                ZB_ZCL_MANUF_CODE_INVALID)}

#define ZB_DECLARE_LEVEL_CLUSTER_LIST(                                     \
    cluster_list_name,                                                     \
    basic_attr_list,                                                       \
    identify_attr_list,                                                    \
    groups_attr_list,                                                      \
    scenes_attr_list,                                                      \
    level_control_attr_list)                                               \
    zb_zcl_cluster_desc_t cluster_list_name[] =                            \
        {                                                                  \
            ZB_ZCL_CLUSTER_DESC(                                           \
                ZB_ZCL_CLUSTER_ID_IDENTIFY,                                \
                ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t),      \
                (identify_attr_list),                                      \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                \
                ZB_ZCL_MANUF_CODE_INVALID),                                \
            ZB_ZCL_CLUSTER_DESC(                                           \
                ZB_ZCL_CLUSTER_ID_BASIC,                                   \
                ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),         \
                (basic_attr_list),                                         \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                \
                ZB_ZCL_MANUF_CODE_INVALID),                                \
            ZB_ZCL_CLUSTER_DESC(                                           \
                ZB_ZCL_CLUSTER_ID_SCENES,                                  \
                ZB_ZCL_ARRAY_SIZE(scenes_attr_list, zb_zcl_attr_t),        \
                (scenes_attr_list),                                        \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                \
                ZB_ZCL_MANUF_CODE_INVALID),                                \
            ZB_ZCL_CLUSTER_DESC(                                           \
                ZB_ZCL_CLUSTER_ID_GROUPS,                                  \
                ZB_ZCL_ARRAY_SIZE(groups_attr_list, zb_zcl_attr_t),        \
                (groups_attr_list),                                        \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                \
                ZB_ZCL_MANUF_CODE_INVALID),                                \
            ZB_ZCL_CLUSTER_DESC(                                           \
                ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,                           \
                ZB_ZCL_ARRAY_SIZE(level_control_attr_list, zb_zcl_attr_t), \
                (level_control_attr_list),                                 \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                \
                ZB_ZCL_MANUF_CODE_INVALID)}

#define ZB_HA_DECLARE_WEATHER_STATION_CLUSTER_LIST(                                  \
    cluster_list_name,                                                               \
    basic_attr_list,                                                                 \
    identify_client_attr_list,                                                       \
    identify_server_attr_list,                                                       \
    temperature_measurement_attr_list,                                               \
    pressure_measurement_attr_list,                                                  \
    humidity_measurement_attr_list)                                                  \
    zb_zcl_cluster_desc_t cluster_list_name[] =                                      \
        {                                                                            \
            ZB_ZCL_CLUSTER_DESC(                                                     \
                ZB_ZCL_CLUSTER_ID_BASIC,                                             \
                ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),                   \
                (basic_attr_list),                                                   \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                          \
                ZB_ZCL_MANUF_CODE_INVALID),                                          \
            ZB_ZCL_CLUSTER_DESC(                                                     \
                ZB_ZCL_CLUSTER_ID_IDENTIFY,                                          \
                ZB_ZCL_ARRAY_SIZE(identify_server_attr_list, zb_zcl_attr_t),         \
                (identify_server_attr_list),                                         \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                          \
                ZB_ZCL_MANUF_CODE_INVALID),                                          \
            ZB_ZCL_CLUSTER_DESC(                                                     \
                ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,                                  \
                ZB_ZCL_ARRAY_SIZE(temperature_measurement_attr_list, zb_zcl_attr_t), \
                (temperature_measurement_attr_list),                                 \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                          \
                ZB_ZCL_MANUF_CODE_INVALID),                                          \
            ZB_ZCL_CLUSTER_DESC(                                                     \
                ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,                              \
                ZB_ZCL_ARRAY_SIZE(pressure_measurement_attr_list, zb_zcl_attr_t),    \
                (pressure_measurement_attr_list),                                    \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                          \
                ZB_ZCL_MANUF_CODE_INVALID),                                          \
            ZB_ZCL_CLUSTER_DESC(                                                     \
                ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,                          \
                ZB_ZCL_ARRAY_SIZE(humidity_measurement_attr_list, zb_zcl_attr_t),    \
                (humidity_measurement_attr_list),                                    \
                ZB_ZCL_CLUSTER_SERVER_ROLE,                                          \
                ZB_ZCL_MANUF_CODE_INVALID),                                          \
            ZB_ZCL_CLUSTER_DESC(                                                     \
                ZB_ZCL_CLUSTER_ID_IDENTIFY,                                          \
                ZB_ZCL_ARRAY_SIZE(identify_client_attr_list, zb_zcl_attr_t),         \
                (identify_client_attr_list),                                         \
                ZB_ZCL_CLUSTER_CLIENT_ROLE,                                          \
                ZB_ZCL_MANUF_CODE_INVALID),                                          \
    }

#define ZB_DECLARE_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num) \
    ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num)

    ZB_DECLARE_SIMPLE_DESC_TYPE(ZB_BOOL_IN_CLUSTER_NUM, ZB_BOOL_OUT_CLUSTER_NUM);
    //ZB_DECLARE_SIMPLE_DESC_TYPE(ZB_CVC_IN_CLUSTER_NUM, ZB_CVC_OUT_CLUSTER_NUM);
    ZB_DECLARE_SIMPLE_DESC_TYPE(ZB_SENSOR_IN_CLUSTER_NUM, ZB_SENSOR_OUT_CLUSTER_NUM);

#define ZB_ZCL_DECLARE_HA_SIMPLE_DESC(ep_name, ep_id, in_clust_num, out_clust_num) \
    ZB_AF_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num)                            \
    simple_desc_##ep_name =                                                        \
        {                                                                          \
            ep_id,                                                                 \
            ZB_AF_HA_PROFILE_ID,                                                   \
            ZB_DEV_ID,                                                             \
            ZB_DEV_VER,                                                            \
            0,                                                                     \
            in_clust_num,                                                          \
            out_clust_num,                                                         \
            {                                                                      \
                ZB_ZCL_CLUSTER_ID_BASIC,                                           \
                ZB_ZCL_CLUSTER_ID_IDENTIFY,                                        \
                ZB_ZCL_CLUSTER_ID_SCENES,                                          \
                ZB_ZCL_CLUSTER_ID_GROUPS,                                          \
            }}

#define ZB_DECLARE_BOOL_EP(ep_name, ep_id, cluster_list)                                              \
    ZB_ZCL_DECLARE_HA_SIMPLE_DESC(ep_name, ep_id,                                                     \
                                  ZB_BOOL_IN_CLUSTER_NUM, ZB_BOOL_OUT_CLUSTER_NUM);                   \
    ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info##ep_name,                                       \
                                       1);                                                            \
    ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID,                                  \
                                0,                                                                    \
                                NULL,                                                                 \
                                ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t), cluster_list, \
                                (zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name,                    \
                                1,                                                                    \
                                reporting_info##ep_name,                                              \
                                0,                                                                    \
                                NULL)

#define ZB_DECLARE_CVC_EP(ep_name, ep_id, cluster_list)                                               \
    ZB_ZCL_DECLARE_HA_SIMPLE_DESC(ep_name, ep_id,                                                     \
                                  ZB_CVC_IN_CLUSTER_NUM, ZB_CVC_OUT_CLUSTER_NUM);                     \
    ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info##ep_name,                                       \
                                       1);                                                            \
    ZBOSS_DEVICE_DECLARE_LEVEL_CONTROL_CTX(cvc_alarm_info##ep_name,                                   \
                                           1);                                                        \
    ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID,                                  \
                                0,                                                                    \
                                NULL,                                                                 \
                                ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t), cluster_list, \
                                (zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name,                    \
                                1,                                                                    \
                                reporting_info##ep_name,                                              \
                                1,                                                                    \
                                cvc_alarm_info##ep_name)

#define ZB_ZCL_DECLARE_WEATHER_STATION_DESC(                                    \
    ep_name,                                                                    \
    ep_id)                                                                      \
    ZB_AF_SIMPLE_DESC_TYPE(ZB_SENSOR_IN_CLUSTER_NUM, ZB_SENSOR_OUT_CLUSTER_NUM) \
    simple_desc_##ep_name =                                                     \
        {                                                                       \
            ep_id,                                                              \
            ZB_AF_HA_PROFILE_ID,                                                \
            ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,                                 \
            0,                                                                  \
            0,                                                                  \
            ZB_SENSOR_IN_CLUSTER_NUM,                                           \
            ZB_SENSOR_OUT_CLUSTER_NUM,                                          \
            {                                                                   \
                ZB_ZCL_CLUSTER_ID_BASIC,                                        \
                ZB_ZCL_CLUSTER_ID_IDENTIFY,                                     \
                ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,                             \
                ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,                         \
                ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,                     \
                ZB_ZCL_CLUSTER_ID_IDENTIFY,                                     \
            }}

#define ZB_HA_DECLARE_WEATHER_STATION_EP(ep_name, ep_id, cluster_list) \
    ZB_ZCL_DECLARE_WEATHER_STATION_DESC(                               \
        ep_name,                                                       \
        ep_id);                                                        \
    ZBOSS_DEVICE_DECLARE_REPORTING_CTX(                                \
        reporting_info##ep_name,                                       \
        ZB_SENSORS_ATTR_COUNT);                                        \
    ZB_AF_DECLARE_ENDPOINT_DESC(                                       \
        ep_name,                                                       \
        ep_id,                                                         \
        ZB_AF_HA_PROFILE_ID,                                           \
        0,                                                             \
        NULL,                                                          \
        ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t),        \
        cluster_list,                                                  \
        (zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name,             \
        ZB_SENSORS_ATTR_COUNT, reporting_info##ep_name, 0, NULL)

#define ZBOSS_DECLARE_DEVICE_CTX_7_EP(device_ctx_name,                   \
                                      ep1_name,                          \
                                      ep2_name,                          \
                                      ep3_name,                          \
                                      ep4_name,                          \
                                      ep5_name,                          \
                                      ep6_name,                          \
                                      ep7_name)                          \
    ZB_AF_START_DECLARE_ENDPOINT_LIST(ep_list_##device_ctx_name)         \
    &ep1_name,                                                           \
        &ep2_name,                                                       \
        &ep3_name,                                                       \
        &ep4_name,                                                       \
        &ep5_name,                                                       \
        &ep6_name,                                                       \
        &ep7_name                                                        \
            ZB_AF_FINISH_DECLARE_ENDPOINT_LIST;                          \
    ZBOSS_DECLARE_DEVICE_CTX(device_ctx_name, ep_list_##device_ctx_name, \
                             (ZB_ZCL_ARRAY_SIZE(ep_list_##device_ctx_name, zb_af_endpoint_desc_t *)))

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

    ZB_DECLARE_BOOL_CLUSTER_LIST(
        fanClusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        fanAttrList);

    ZB_DECLARE_BOOL_EP(
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

    ZB_DECLARE_CVC_EP(
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

    ZB_DECLARE_CVC_EP(
        led2Ep,
        ZB_LED2_ENDPOINT,
        led2Clusters);

    // LED strategy
    ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
        ledStrategyAttrList,
        &iface.getContext().ledStrategyAttr.on_off);

    ZB_DECLARE_BOOL_CLUSTER_LIST(
        ledStrategyClusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        ledStrategyAttrList);

    ZB_DECLARE_BOOL_EP(
        ledStrategyEp,
        ZB_STRATEGY_ENDPOINT,
        ledStrategyClusters);

    // Relay water.
    ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
        waterAttrList,
        &iface.getContext().waterAttr.on_off);

    ZB_DECLARE_BOOL_CLUSTER_LIST(
        waterClusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        waterAttrList);

    ZB_DECLARE_BOOL_EP(
        relayWaterEp,
        ZB_WATER_ENDPOINT,
        waterClusters);

    // Relay nutri.
    ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
        nutriAttrList,
        &iface.getContext().nutriAttr.on_off);

    ZB_DECLARE_BOOL_CLUSTER_LIST(
        nutriClusters,
        basicAttrList,
        identifyAttrList,
        groupsAttrList,
        scenesAttrList,
        nutriAttrList);

    ZB_DECLARE_BOOL_EP(
        relayNutriEp,
        ZB_NUTRI_ENDPOINT,
        nutriClusters);

    // Sensors.
    ZB_ZCL_DECLARE_TEMP_MEASUREMENT_ATTRIB_LIST(
        tempAttrList,
        &iface.getContext().tempAttr.measure_value,
        &iface.getContext().tempAttr.min_measure_value,
        &iface.getContext().tempAttr.max_measure_value,
        &iface.getContext().tempAttr.tolerance);

    ZB_ZCL_DECLARE_PRESSURE_MEASUREMENT_ATTRIB_LIST(
        pressAttrList,
        &iface.getContext().pressAttr.measure_value,
        &iface.getContext().pressAttr.min_measure_value,
        &iface.getContext().pressAttr.max_measure_value,
        &iface.getContext().pressAttr.tolerance);

    ZB_ZCL_DECLARE_REL_HUMIDITY_MEASUREMENT_ATTRIB_LIST(
        humAttrList,
        &iface.getContext().humAttr.measure_value,
        &iface.getContext().humAttr.min_measure_value,
        &iface.getContext().humAttr.max_measure_value);

    ZB_ZCL_DECLARE_IDENTIFY_CLIENT_ATTRIB_LIST(
        identifyClientAttrList);

    ZB_HA_DECLARE_WEATHER_STATION_CLUSTER_LIST(
        sensorClusters,
        basicAttrList,
        identifyClientAttrList,
        identifyAttrList,
        tempAttrList,
        pressAttrList,
        humAttrList);

    ZB_HA_DECLARE_WEATHER_STATION_EP(
        sensorsEp,
        ZB_SENSORS_ENDPOINT,
        sensorClusters);

    // Endpoint declatarion.
    ZBOSS_DECLARE_DEVICE_CTX_7_EP(
        ctrlCtx,
        fanEp,
        led1Ep,
        led2Ep,
        ledStrategyEp,
        relayWaterEp,
        relayNutriEp,
        sensorsEp);
}

extern "C"
{
    void zboss_signal_handler(zb_bufid_t bufid)
    {
        zb_zdo_app_signal_hdr_t *signal_header = NULL;
        zb_zdo_app_signal_type_t signal = zb_get_app_signal(bufid, &signal_header);
        zb_ret_t err;

        switch (signal)
        {
        case ZB_ZDO_SIGNAL_SKIP_STARTUP:
            err = ZB_SCHEDULE_APP_ALARM(iface.sensorReadCallback,
                                        0,
                                        ZB_MILLISECONDS_TO_BEACON_INTERVAL(
                                            ZB_SENSORS_CHECK_PERIOS_MS));
            if (err)
            {
                LOG_ERR("Failed to schedule app alarm (error code: %d)", err);
            }
            break;

        default:
            break;
        }

        ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

        if (bufid)
        {
            zb_buf_free(bufid);
        }
    }
}