#pragma once

#define BASIC_APP_VER 1
#define BASIC_ZB_STACK_VER 10
#define BASIC_HW_VER 1
#define BASIC_MANUF_NAME "Nordic"
#define BASIC_MODEL_ID "FC_v0.1"
#define BASIC_DATE_CODE "20220914"
#define BASIC_POWER_SOURCE ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE
#define BASIC_LOCATION "Field"
#define BASIC_ENV ZB_ZCL_BASIC_ENV_UNSPECIFIED

#define ZIGBEE_NETWORK_STATE_LED DK_LED2
#define IDENTIFY_LED DK_LED3
#define IDENTIFY_MODE_BUTTON DK_BTN2_MSK
#define FACTORY_RESET_BUTTON IDENTIFY_MODE_BUTTON

#define ZB_FAN_ENDPOINT 10
#define ZB_LED1_ENDPOINT 11
#define ZB_LED2_ENDPOINT 12
#define ZB_STRATEGY_ENDPOINT 13
#define ZB_WATER_ENDPOINT 14
#define ZB_NUTRI_ENDPOINT 15

#define ZB_DEV_ID 0x2137
#define ZB_DEV_VER 1

#define ZB_IN_CLUSTER_NUM 10
#define ZB_OUT_CLUSTER_NUM 0 // TODO: Change
//#define ZB_TOTAL_CLUSTER_NUM (ZB_IN_CLUSTER_NUM + ZB_OUT_CLUSTER_NUM)

// TODO: Change???
#define ZB_REPORT_ATTR_COUNT (ZB_ZCL_ON_OFF_REPORT_ATTR_COUNT + ZB_ZCL_LEVEL_CONTROL_REPORT_ATTR_COUNT)

// TODO: Change???
#define ZB_CVC_ATTR_COUNT 1

/**
 * @brief Declare cluster list for on / off device
 * @param cluster_list_name - cluster list variable name
 * @param basic_attr_list - attribute list for Basic cluster
 * @param identify_attr_list - attribute list for Identify cluster
 * @param groups_attr_list - attribute list for Groups cluster
 * @param scenes_attr_list - attribute list for Scenes cluster
 * @param on_off_attr_list - attribute list for On/Off cluster
 */
#define ZB_DECLARE_ON_OFF_CLUSTER_LIST(                               \
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

/**
 * @brief Declare simple descriptor for device
 * @param ep_name - endpoint variable name
 * @param ep_id - endpoint ID
 * @param in_clust_num - number of supported input clusters
 * @param out_clust_num - number of supported output clusters
 */
#define ZB_ZCL_DECLARE_HA_SIMPLE_DESC(ep_name, ep_id, in_clust_num, out_clust_num) \
    ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num);                           \
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
#define ZB_DECLARE_EP(ep_name, ep_id, cluster_list)                                                   \
    ZB_ZCL_DECLARE_HA_SIMPLE_DESC(ep_name, ep_id,                                                     \
                                  ZB_IN_CLUSTER_NUM, ZB_OUT_CLUSTER_NUM);                             \
    ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info##ep_name,                                       \
                                       ZB_REPORT_ATTR_COUNT);                                         \
    ZBOSS_DEVICE_DECLARE_LEVEL_CONTROL_CTX(cvc_alarm_info##ep_name,                                   \
                                           ZB_CVC_ATTR_COUNT);                                        \
    ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID,                                  \
                                0,                                                                    \
                                NULL,                                                                 \
                                ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t), cluster_list, \
                                (zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name,                    \
                                ZB_REPORT_ATTR_COUNT,                                                 \
                                reporting_info##ep_name,                                              \
                                ZB_CVC_ATTR_COUNT,                                                    \
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