#pragma once
#include "types/inference_types.hpp"

namespace app
{
    namespace inference
    {
        namespace classifiers
        {
            class Classifier
            {
            private:
                uint16_t m_cntr = 0;
                //signal_t featureSig;

                int loadData(std::size_t offset, std::size_t length, float *out_ptr);

                float m_temp;
                float m_press;
                float m_hum;
                float m_gas;

            public:
                void init();
                Classification readNetwork(float temp, float press, float hum, float gas);
            };
        }
    }
}