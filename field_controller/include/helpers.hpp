#pragma once
#include <math.h>

namespace app
{
    namespace helpers
    {
        double exponential(double val, double minv, double maxv);

        class LowPassFilter
        {
        private:
            double previousOutput = 0;
            const double smoothingFactor = 1;

        public:
            LowPassFilter(double _sf = 1);
            void reset();
            double read(double input);
        };
    }
}