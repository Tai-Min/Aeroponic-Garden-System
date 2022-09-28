#pragma once
#include <cstdint>

namespace app
{
    namespace networking
    {
        namespace zigbee
        {
            using IdentifyCallback = void (*)();
            using Callback = void (*)(uint16_t);

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

            struct FeedbackType
            {
                enum class Values : uint8_t
                {
                    LIGHT_SENSOR_1,
                    LIGHT_SENSOR_2,
                    HUMIDITY,
                    TEMPERATURE,
                    PRESSURE,
                    GAS,
                };
                static constexpr uint8_t length() { return 6; }
            };
        }
    }
}