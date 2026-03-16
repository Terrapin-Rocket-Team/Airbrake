#ifndef ASTRA_ROCKET_BR_ACCEL_H
#define ASTRA_ROCKET_BR_ACCEL_H

#include <Sensors/Accel/Accel.h>

#include "BlueRaven.h"

namespace astra_rocket
{
    class BRAccel : public astra::Accel
    {
    public:
        explicit BRAccel(BlueRaven &parent, const char *name = "BlueRaven Accel");

        bool shouldUpdate(double currentTime) override;

    protected:
        int init() override;
        int read() override;

    private:
        BlueRaven *parent = nullptr;
        uint32_t lastSampleCount = 0;
    };
}

#endif // ASTRA_ROCKET_BR_ACCEL_H
