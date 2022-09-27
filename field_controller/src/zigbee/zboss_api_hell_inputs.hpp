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

#include "config.hpp"

LOG_MODULE_REGISTER(zigbee, LOG_LEVEL_DBG);

namespace
{
    /**
     * @brief Declare cluster list for on / off device
     * @param cluster_list_name - cluster list variable name
     * @param basic_attr_list - attribute list for Basic cluster
     * @param identify_attr_list - attribute list for Identify cluster
     * @param groups_attr_list - attribute list for Groups cluster
     * @param scenes_attr_list - attribute list for Scenes cluster
     * @param on_off_attr_list - attribute list for On/Off cluster
     */
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

/**
 * @brief Declare cluster list for level control device
 * @param cluster_list_name - cluster list variable name
 * @param basic_attr_list - attribute list for Basic cluster
 * @param identify_attr_list - attribute list for Identify cluster
 * @param groups_attr_list - attribute list for Groups cluster
 * @param scenes_attr_list - attribute list for Scenes cluster
 * @param level_control_attr_list - attribute list for On/Off cluster
 */
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

#define ZB_DECLARE_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num) \
    ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num)

    ZB_DECLARE_SIMPLE_DESC_TYPE(ZB_IN_CLUSTER_NUM, ZB_OUT_CLUSTER_NUM);

/**
 * @brief Declare simple descriptor for device
 * @param ep_name - endpoint variable name
 * @param ep_id - endpoint ID
 * @param in_clust_num - number of supported input clusters
 * @param out_clust_num - number of supported output clusters
 */
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

/**
 * @brief Declare endpoint for Dimmable Light device
 * @param ep_name - endpoint variable name
 * @param ep_id - endpoint ID
 * @param cluster_list - endpoint cluster list
 */
#define ZB_DECLARE_BOOL_EP(ep_name, ep_id, cluster_list)                                              \
    ZB_ZCL_DECLARE_HA_SIMPLE_DESC(ep_name, ep_id,                                                     \
                                  ZB_IN_CLUSTER_NUM, ZB_OUT_CLUSTER_NUM);                             \
    ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info##ep_name,                                       \
                                       1);                                                            \
    ZBOSS_DEVICE_DECLARE_LEVEL_CONTROL_CTX(cvc_alarm_info##ep_name,                                   \
                                           0);                                                        \
    ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID,                                  \
                                0,                                                                    \
                                NULL,                                                                 \
                                ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t), cluster_list, \
                                (zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name,                    \
                                1,                                                                    \
                                reporting_info##ep_name,                                              \
                                0,                                                                    \
                                cvc_alarm_info##ep_name)

#define ZB_DECLARE_CVC_EP(ep_name, ep_id, cluster_list)                                               \
    ZB_ZCL_DECLARE_HA_SIMPLE_DESC(ep_name, ep_id,                                                     \
                                  ZB_IN_CLUSTER_NUM, ZB_OUT_CLUSTER_NUM);                             \
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
/**
  Declare application's device context for six-endpoint device

  @note Device has an additional Green Power endpoint if it is ZC or ZR
  and GPPB feature (Mandatory for Zigbee 3.0 ZC/ZR) is enabled.

  @param device_ctx_name - device context variable name
  @param ep1_name - variable holding context for endpoint 1
  @param ep2_name - variable holding context for endpoint 2
  @param ep3_name - variable holding context for endpoint 3
  @param ep4_name - variable holding context for endpoint 4
  @param ep5_name - variable holding context for endpoint 5
  @param ep6_name - variable holding context for endpoint 6
*/
#define ZBOSS_DECLARE_DEVICE_CTX_6_EP(device_ctx_name,                   \
                                      ep1_name,                          \
                                      ep2_name,                          \
                                      ep3_name,                          \
                                      ep4_name,                          \
                                      ep5_name,                          \
                                      ep6_name)                          \
    ZB_AF_START_DECLARE_ENDPOINT_LIST(ep_list_##device_ctx_name)         \
    &ep1_name,                                                           \
        &ep2_name,                                                       \
        &ep3_name,                                                       \
        &ep4_name,                                                       \
        &ep5_name,                                                       \
        &ep6_name,                                                       \
        ZB_AF_FINISH_DECLARE_ENDPOINT_LIST;                              \
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