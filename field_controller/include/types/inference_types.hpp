#pragma once

#include <cstdint>

namespace app
{
    namespace inference
    {
        namespace classifiers
        {
            enum class Classification : uint8_t {
                GOOD = 0,
                MOULD
            };
        }
    }
}