#pragma once
#include <cstdint>

namespace app
{
    namespace io
    {
        namespace controllers
        {

            /**
             * @brief Defines how to control actuators.
             */
            struct ClosedControlStrategy
            {
                enum class Value : bool
                {
                    SELF,  //!< Controlled by this controller based on sensor measurement.
                    ZIGBEE //!< Controlled by global controller via Zigbee commands.
                };
                constexpr uint8_t length() { return 2; }
            };
        }
    }
}