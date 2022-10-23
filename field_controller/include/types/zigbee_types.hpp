#pragma once

#include <cstdint>
extern "C"
{
#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zcl/zb_zcl_temp_measurement_addons.h>
}

namespace app
{
    namespace networking
    {
        namespace zigbee
        {
            struct CallbackType
            {
                enum class Values : uint8_t
                {
                    FAN,
                    RELAY_WATER,
                    RELAY_NUTRI,
                    LED_SIDE_1,
                    LED_SIDE_2,
                    LED_STRATEGY,
                };
                static constexpr uint8_t length() { return 7; }
            };

            struct SensorResult
            {
                bool ok;
                float temperature;
                float pressure;
                float humidity;
                float gasResistance;
                uint8_t classification;
            };

            struct PressureAttr
            {
                zb_int16_t measure_value;
                zb_int16_t min_measure_value;
                zb_int16_t max_measure_value;
                zb_uint16_t tolerance;
            };

            struct HumidityAttr
            {
                zb_int16_t measure_value;
                zb_int16_t min_measure_value;
                zb_int16_t max_measure_value;
            };

            struct GasResistanceAttr
            {
                zb_int16_t measure_value;
                zb_int16_t min_measure_value;
                zb_int16_t max_measure_value;
            };

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

                zb_zcl_temp_measurement_attrs_t tempAttr;
                PressureAttr pressAttr;
                HumidityAttr humAttr;

                zb_zcl_on_off_attrs_t classificationAttr;
            };

            using Callback = void (*)(uint16_t);
            using MeasurementCallback = SensorResult (*)();
        }
    }
}