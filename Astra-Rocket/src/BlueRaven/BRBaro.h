#ifndef ASTRA_ROCKET_BR_BARO_H
#define ASTRA_ROCKET_BR_BARO_H

#include <Sensors/Baro/Barometer.h>

#include "BlueRaven.h"

namespace astra_rocket
{
    class BRBaro : public astra::Barometer
    {
    public:
        explicit BRBaro(BlueRaven &parent, const char *name = "BlueRaven Baro");

        double getAltitudeAglM() const { return altitudeAglM; }

    protected:
        int init() override;
        int read() override;

    private:
        BlueRaven *parent = nullptr;
        double altitudeAglM = 0.0;
    };
}

#endif // ASTRA_ROCKET_BR_BARO_H
