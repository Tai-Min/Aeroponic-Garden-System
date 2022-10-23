#include "classifier.hpp"
#include <zephyr/logging/log.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

#include "config/software.hpp"

LOG_MODULE_REGISTER(classifier, CLASSIFIER_LOG_LVL);

namespace
{
    signal_t featureSig;
}

namespace app
{
    namespace inference
    {
        namespace classifiers
        {
            int Classifier::loadData(std::size_t offset, std::size_t length, float *out_ptr)
            {
                out_ptr[0] = m_temp;
                out_ptr[1] = m_press;
                out_ptr[2] = m_hum;
                out_ptr[3] = m_gas;
            }

            void Classifier::init()
            {
                LOG_INF("Classifier initializing");
                featureSig.total_length = 4;
                featureSig.get_data = [=](std::size_t offset, std::size_t length, float *out_ptr) -> int
                {
                    return loadData(offset, length, out_ptr);
                };
                LOG_INF("Classifier initialized");
            }

            Classification Classifier::readNetwork(float temp, float press, float hum, float gas)
            {
                m_temp = temp;
                m_press = press;
                m_hum = hum;
                m_gas = gas;

                ei_impulse_result_t classificationResult = {0};

                run_classifier(&featureSig, &classificationResult);

                float good = classificationResult.classification[static_cast<uint8_t>(Classification::GOOD)].value;
                float mould = classificationResult.classification[static_cast<uint8_t>(Classification::MOULD)].value;

                LOG_DBG("Classification result: GOOD %f, MOULD %f", good, mould);

                if (good > 0.8 && m_cntr > 0)
                {
                    m_cntr--;
                }
                else if (mould > 0.8)
                {
                    m_cntr++;
                }

                LOG_DBG("Classifier reading err cntr %d/100", m_cntr);

                if (m_cntr >= 100)
                {
                    return Classification::MOULD;
                }
                return Classification::GOOD;
            }
        }
    }
}