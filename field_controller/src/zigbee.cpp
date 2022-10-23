#include "zigbee/zboss_api_hell.hpp"
#include "config/hardware.hpp"

namespace
{
    constexpr struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZIGBEE_IDENTIFY_LED_PIN, gpios);
}

namespace app
{
    namespace networking
    {
        namespace zigbee
        {
            bool ZigbeeInterface::m_isInit = false;
            app::io::digital::Output ZigbeeInterface::m_identifyLed = app::io::digital::Output(led, "Identify LED");
            DeviceContext ZigbeeInterface::m_devCtx;
            ZigbeeInterface ZigbeeInterface::m_instance = ZigbeeInterface();
            bool ZigbeeInterface::m_isBlinking = false;
            Callback ZigbeeInterface::m_callbacks[CallbackType::length()] = {nullptr};
            MeasurementCallback ZigbeeInterface::m_sensorCallback = nullptr;

            /*void ZigbeeInterface::startIdentify(zb_bufid_t bufid)
             {
                 if (ZB_JOINED())
                 {
                     // If is in identifying mode.
                     if (m_devCtx.identify.identify_time == ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE)
                     {
                         LOG_INF("Start identifying");

                         zb_ret_t err = zb_bdb_finding_binding_target(ZB_FAN_ENDPOINT);

                         if (err == RET_OK)
                         {
                             LOG_INF("Entering identify mode");
                         }
                         else if (err == RET_INVALID_STATE)
                         {
                             LOG_WRN("RET_INVALID_STATE - Cannot enter identify mode");
                         }
                         else
                         {
                             ZB_ERROR_CHECK(err);
                         }
                     }
                     else
                     {
                         LOG_INF("Canceling identify mode");
                         zb_bdb_finding_binding_target_cancel();
                     }
                 }
                 else
                 {
                     LOG_WRN("Device not in a network - cannot enter identify mode");
                 }
             }*/

            void ZigbeeInterface::btnCallback(uint32_t button_state, uint32_t has_changed)
            {
                if (ZIGBEE_IDENTIFY_MODE_BUTTON & has_changed)
                {
                    if (!(ZIGBEE_IDENTIFY_MODE_BUTTON & button_state))
                    {
                        if (was_factory_reset_done())
                        {
                            LOG_INF("Factory reset done");
                        }
                        else
                        {
                            // ZB_SCHEDULE_APP_CALLBACK(ZigbeeInterface::startIdentify, 0);

                            user_input_indicate();
                        }
                    }
                }
                check_factory_reset_button(button_state, has_changed);
            }

            void ZigbeeInterface::sensorReadCallback(zb_bufid_t bufid)
            {
                LOG_DBG("Sensor read callback called");

                if (m_sensorCallback == nullptr)
                {
                    LOG_WRN("Sensor read callback is null - skipping");
                }
                else
                {
                    SensorResult res = m_sensorCallback();

                    if (res.ok)
                    {
                        setTemperature(res.temperature * ZB_SENSORS_TEMP_MULTIPLIER);
                        setPressure(res.pressure * ZB_SENSORS_PRESS_MULTIPLIER);
                        setHumidity(res.humidity * ZB_SENSORS_HUM_MULTIPLIER);
                        setClassification(res.classification);
                    }
                    else
                    {
                        LOG_WRN("Failed to perform sensor read");
                    }
                }

                zb_ret_t zb_err = ZB_SCHEDULE_APP_ALARM(sensorReadCallback,
                                                        0,
                                                        ZB_MILLISECONDS_TO_BEACON_INTERVAL(
                                                            ZB_SENSORS_CHECK_PERIOS_MS));
                if (zb_err)
                {
                    LOG_ERR("Failed to schedule app alarm (error code: %d)", zb_err);
                }
            }

            void ZigbeeInterface::setTemperature(int16_t temp)
            {
                LOG_INF("Setting temperature variable to %d", temp);

                zb_zcl_status_t status = zb_zcl_set_attr_val(
                    ZB_SENSORS_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                    (zb_uint8_t *)&temp,
                    ZB_FALSE);

                if (status)
                {
                    LOG_ERR("Failed to set temperature ZCL attribute (error code: %d)", status);
                }
            }

            void ZigbeeInterface::setPressure(int16_t press)
            {
                LOG_INF("Setting pressure variable to %d", press);

                zb_zcl_status_t status = zb_zcl_set_attr_val(
                    ZB_SENSORS_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
                    (zb_uint8_t *)&press,
                    ZB_FALSE);

                if (status)
                {
                    LOG_ERR("Failed to set pressure ZCL attribute (error code: %d)", status);
                }
            }

            void ZigbeeInterface::setHumidity(int16_t hum)
            {
                LOG_INF("Setting humidity variable to %d", hum);

                zb_zcl_status_t status = zb_zcl_set_attr_val(
                    ZB_SENSORS_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                    (zb_uint8_t *)&hum,
                    ZB_FALSE);
                if (status)
                {
                    LOG_ERR("Failed to set humidity ZCL attribute (error code: %d)", status);
                }
            }

            void ZigbeeInterface::setClassification(bool c) {
                LOG_INF("Setting classification variable to %d", c);

                zb_zcl_status_t status = zb_zcl_set_attr_val(
                    ZB_CLASSIFIER_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_ON_OFF,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                    (zb_uint8_t *)&c,
                    ZB_FALSE);
                if (status)
                {
                    LOG_ERR("Failed to set classification ZCL attribute (error code: %d)", status);
                }
            }

            void ZigbeeInterface::maybeSetBoolValue(zb_zcl_device_callback_param_t *device_cb_param, Callback cb)
            {

                if (device_cb_param->device_cb_id == ZB_ZCL_SET_ATTR_VALUE_CB_ID)
                {
                    zb_uint8_t cluster_id = device_cb_param->cb_param.set_attr_value_param.cluster_id;
                    zb_uint8_t attr_id = device_cb_param->cb_param.set_attr_value_param.attr_id;

                    if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF)
                    {
                        bool value = device_cb_param->cb_param.set_attr_value_param.values.data8;

                        if (attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
                        {
                            ZB_ZCL_SET_ATTRIBUTE(
                                device_cb_param->endpoint,
                                ZB_ZCL_CLUSTER_ID_ON_OFF,
                                ZB_ZCL_CLUSTER_SERVER_ROLE,
                                ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                (zb_uint8_t *)&value,
                                ZB_FALSE);

                            cb(value);
                        }
                    }
                    else
                    {
                        LOG_INF("Unhandled cluster attribute for boolean callback (id: %d), skipping", cluster_id);
                        device_cb_param->status = RET_NOT_IMPLEMENTED;
                    }
                }
            }

            void ZigbeeInterface::maybeSetLevelValue(zb_zcl_device_callback_param_t *device_cb_param, Callback cb)
            {
                zb_uint8_t value;
                zb_uint8_t cluster_id = device_cb_param->cb_param.set_attr_value_param.cluster_id;
                zb_uint8_t attr_id = device_cb_param->cb_param.set_attr_value_param.attr_id;

                if (device_cb_param->device_cb_id == ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID)
                {
                    value = device_cb_param->cb_param.level_control_set_value_param.new_value;
                }
                else if (device_cb_param->device_cb_id == ZB_ZCL_SET_ATTR_VALUE_CB_ID &&
                         cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL &&
                         attr_id == ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID)
                {
                    value = device_cb_param->cb_param.set_attr_value_param.values.data8;
                }
                else
                {
                    LOG_INF("Unhandled cluster attribute for level callback (id: %d), skipping", cluster_id);
                    device_cb_param->status = RET_NOT_IMPLEMENTED;
                    return;
                }

                ZB_ZCL_SET_ATTRIBUTE(
                    device_cb_param->endpoint,
                    ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
                    (zb_uint8_t *)&value,
                    ZB_FALSE);

                cb(value);
            }

            void ZigbeeInterface::zclDeviceCallback(zb_bufid_t bufid)
            {
                zb_zcl_device_callback_param_t *device_cb_param =
                    ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

                device_cb_param->status = RET_OK;

                switch (device_cb_param->endpoint)
                {
                case ZB_FAN_ENDPOINT:
                    maybeSetBoolValue(device_cb_param, m_callbacks[static_cast<uint8_t>(CallbackType::Values::FAN)]);
                    break;
                case ZB_LED1_ENDPOINT:
                    maybeSetLevelValue(device_cb_param, m_callbacks[static_cast<uint8_t>(CallbackType::Values::FAN)]);
                    break;
                case ZB_LED2_ENDPOINT:
                    maybeSetLevelValue(device_cb_param, m_callbacks[static_cast<uint8_t>(CallbackType::Values::FAN)]);
                    break;
                case ZB_STRATEGY_ENDPOINT:
                    maybeSetBoolValue(device_cb_param, m_callbacks[static_cast<uint8_t>(CallbackType::Values::LED_STRATEGY)]);
                    break;
                case ZB_WATER_ENDPOINT:
                    maybeSetBoolValue(device_cb_param, m_callbacks[static_cast<uint8_t>(CallbackType::Values::RELAY_WATER)]);
                    break;
                case ZB_NUTRI_ENDPOINT:
                    maybeSetBoolValue(device_cb_param, m_callbacks[static_cast<uint8_t>(CallbackType::Values::RELAY_NUTRI)]);
                    break;
                default:
                    if (zcl_scenes_cb(bufid) == ZB_FALSE)
                    {
                        device_cb_param->status = RET_NOT_IMPLEMENTED;
                    }
                    break;
                }
            }

            void ZigbeeInterface::toggleIdentify(zb_bufid_t bufid)
            {
                static bool blink = false;
                m_identifyLed.setState(blink);
                blink = !blink;
                ZB_SCHEDULE_APP_ALARM(toggleIdentify, bufid, ZB_MILLISECONDS_TO_BEACON_INTERVAL(500));
            }

            void ZigbeeInterface::identifyCallback(zb_bufid_t bufid)
            {
                if (bufid)
                {
                    m_isBlinking = true;
                    ZB_SCHEDULE_APP_CALLBACK(toggleIdentify, bufid);
                }
                else
                {
                    m_isBlinking = false;
                    ZB_SCHEDULE_APP_ALARM_CANCEL(toggleIdentify, ZB_ALARM_ANY_PARAM);
                    m_identifyLed.setState(false);
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

                m_devCtx.fanAttr.on_off = static_cast<zb_bool_t>(ZB_ZCL_ON_OFF_IS_OFF);

                m_devCtx.led1Attr.current_level = ZB_ZCL_LEVEL_CONTROL_LEVEL_MIN_VALUE;
                m_devCtx.led1Attr.remaining_time = ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;

                m_devCtx.led2Attr.current_level = ZB_ZCL_LEVEL_CONTROL_LEVEL_MIN_VALUE;
                m_devCtx.led2Attr.remaining_time = ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;

                m_devCtx.ledStrategyAttr.on_off = static_cast<zb_bool_t>(ZB_ZCL_ON_OFF_IS_OFF);
                m_devCtx.waterAttr.on_off = static_cast<zb_bool_t>(ZB_ZCL_ON_OFF_IS_OFF);
                m_devCtx.nutriAttr.on_off = static_cast<zb_bool_t>(ZB_ZCL_ON_OFF_IS_OFF);

                m_devCtx.tempAttr.measure_value = ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_UNKNOWN;
                m_devCtx.tempAttr.min_measure_value = SENSOR_TEMP_CELSIUS_MIN;
                m_devCtx.tempAttr.max_measure_value = SENSOR_TEMP_CELSIUS_MAX;
                m_devCtx.tempAttr.tolerance = SENSOR_TEMP_CELSIUS_TOLERANCE;

                m_devCtx.pressAttr.measure_value = ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_UNKNOWN;
                m_devCtx.pressAttr.min_measure_value = SENSOR_PRESSURE_KPA_MIN;
                m_devCtx.pressAttr.max_measure_value = SENSOR_PRESSURE_KPA_MAX;
                m_devCtx.pressAttr.tolerance = SENSOR_PRESSURE_KPA_TOLERANCE;

                m_devCtx.humAttr.measure_value = ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_UNKNOWN;
                m_devCtx.humAttr.min_measure_value = SENSOR_HUMIDITY_PERCENT_MIN;
                m_devCtx.humAttr.max_measure_value = SENSOR_HUMIDITY_PERCENT_MAX;

                m_devCtx.classificationAttr.on_off = static_cast<zb_bool_t>(ZB_ZCL_ON_OFF_IS_OFF);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_FAN_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_ON_OFF,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                    static_cast<zb_uint8_t *>(&m_devCtx.fanAttr.on_off),
                    ZB_FALSE);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_LED1_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
                    static_cast<zb_uint8_t *>(&m_devCtx.led1Attr.current_level),
                    ZB_FALSE);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_LED2_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
                    static_cast<zb_uint8_t *>(&m_devCtx.led2Attr.current_level),
                    ZB_FALSE);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_STRATEGY_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_ON_OFF,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                    static_cast<zb_uint8_t *>(&m_devCtx.ledStrategyAttr.on_off),
                    ZB_FALSE);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_WATER_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_ON_OFF,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                    static_cast<zb_uint8_t *>(&m_devCtx.waterAttr.on_off),
                    ZB_FALSE);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_NUTRI_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_ON_OFF,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                    static_cast<zb_uint8_t *>(&m_devCtx.nutriAttr.on_off),
                    ZB_FALSE);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_SENSORS_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                    reinterpret_cast<zb_uint8_t *>(&m_devCtx.tempAttr.measure_value),
                    ZB_FALSE);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_SENSORS_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
                    reinterpret_cast<zb_uint8_t *>(&m_devCtx.pressAttr.measure_value),
                    ZB_FALSE);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_SENSORS_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                    reinterpret_cast<zb_uint8_t *>(&m_devCtx.humAttr.measure_value),
                    ZB_FALSE);

                ZB_ZCL_SET_ATTRIBUTE(
                    ZB_CLASSIFIER_ENDPOINT,
                    ZB_ZCL_CLUSTER_ID_ON_OFF,
                    ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                    static_cast<zb_uint8_t *>(&m_devCtx.classificationAttr.on_off),
                    ZB_FALSE);
            }

            bool ZigbeeInterface::init() const
            {
                LOG_INF("Zigbee initializing");

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

                bool ok = m_identifyLed.init();
                if (!ok)
                {
                    LOG_ERR("Identify LED init failed");
                    return false;
                }

                register_factory_reset_button(ZIGBEE_FACTORY_RESET_BUTTON);

                ZB_ZCL_REGISTER_DEVICE_CB(zclDeviceCallback);
                ZB_AF_REGISTER_DEVICE_CTX(&ctrlCtx);

                clusterAttrInit();

                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_FAN_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_LED1_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_LED2_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_STRATEGY_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_WATER_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_NUTRI_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_SENSORS_ENDPOINT, identifyCallback);
                ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(ZB_CLASSIFIER_ENDPOINT, identifyCallback);

                zcl_scenes_init();

                m_isInit = true;

                LOG_INF("Zigbee initialized");
                return true;
            }

            void ZigbeeInterface::registerCallback(CallbackType::Values type, Callback callback)
            {
                m_callbacks[static_cast<uint8_t>(type)] = callback;
            };

            void ZigbeeInterface::registerSensorCallback(MeasurementCallback cb)
            {
                m_sensorCallback = cb;
            }

            void ZigbeeInterface::doWorkDetached() const
            {
                zigbee_enable();
                LOG_INF("Worker started");
            }

            bool ZigbeeInterface::ok() const
            {
                return true;
            }

            constexpr const DeviceContext &ZigbeeInterface::getContext()
            {
                return m_devCtx;
            }

            void ZigbeeInterface::cleanup()
            {
                if (m_identifyLed.ok() && m_isBlinking)
                {
                    identifyCallback(0);
                }
            }

            bool ZigbeeInterface::isIdentifying()
            {
                return m_isBlinking;
            }
        }
    }
}