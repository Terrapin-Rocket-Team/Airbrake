#ifndef ASTRA_ROCKET_BR_HG_ACCEL_H
#define ASTRA_ROCKET_BR_HG_ACCEL_H

#include <Sensors/Accel/Accel.h>

#include "BlueRaven.h"

namespace astra_rocket
{
    class BGHGAccel : public astra::Accel
    {
    public:
        explicit BGHGAccel(BlueRaven &parent, const char *name = "BlueRaven Hi-G");

        bool shouldUpdate(double currentTime) override;

    protected:
        int init() override;
        int read() override;

    private:
        BlueRaven *parent = nullptr;
        uint32_t lastSampleCount = 0;
    };
}

#endif // ASTRA_ROCKET_BR_HG_ACCEL_H
