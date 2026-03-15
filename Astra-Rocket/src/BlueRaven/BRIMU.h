#ifndef ASTRA_ROCKET_BR_IMU_H
#define ASTRA_ROCKET_BR_IMU_H

#include <Sensors/IMU/IMU6DoF.h>

#include "BlueRaven.h"

namespace astra_rocket
{
    class BRIMU : public astra::IMU6DoF
    {
    public:
        explicit BRIMU(BlueRaven &parent, const char *name = "BlueRaven IMU");

    protected:
        int init() override;
        int read() override;

    private:
        BlueRaven *parent = nullptr;
    };
}

#endif // ASTRA_ROCKET_BR_IMU_H
