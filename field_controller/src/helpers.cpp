#include "helpers.hpp"

#include <math.h>

namespace app
{
    namespace helpers
    {
        double exponential(double val, double minv, double maxv)
        {
            if (val == minv)
            {
                return minv;
            }

            else if (val == maxv)
            {
                return maxv;
            }

            else if (minv < 0 || maxv < 0)
            {
                return 0;
            }

            double expMin = log(minv + 1);
            double expMax = log(maxv + 1);
            double a = (expMax - expMin) / (maxv - minv);
            double b = expMin;

            double expVal = a * val + b;
            return exp(expVal) - 1;
        }

        LowPassFilter::LowPassFilter(double _sf) : smoothingFactor(_sf){};

        void LowPassFilter::reset()
        {
            previousOutput = 0;
        }

        double LowPassFilter::read(double input)
        {
            double output = previousOutput + smoothingFactor * (input - previousOutput);
            previousOutput = output;
            return output;
        }
    }
}