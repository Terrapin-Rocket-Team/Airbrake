#include "ErrorCorrectedBaro.h"

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace astra {

ErrorCorrectedBaro::ErrorCorrectedBaro(const char *name)
    : Barometer(name) {
    addColumn("%0.3f", &altitudeDelta, "Alt Corr (m)");
    addColumn("%0.1f", &dynamicPressureHpa, "Dyn Press (hPa)");
    addColumn("%0.1f", &correctionAngleDeg, "Corr Angle (deg)");
    refreshAlpha();
}

ErrorCorrectedBaro::ErrorCorrectedBaro(Barometer *inner, const char *name)
    : ErrorCorrectedBaro(name) {
    setInnerBarometer(inner);
}

void ErrorCorrectedBaro::setInnerBarometer(Barometer *inner) {
    if (inner == this) {
        return;
    }
    innerBaro = inner;
    if (innerBaro) {
        innerBaro->setAutoUpdate(false);
    }
}

void ErrorCorrectedBaro::setCorrectionEnabled(bool enabled) {
    correctionEnabled = enabled;
    if (!correctionEnabled) {
        altitudeDelta = 0.0;
    }
}

void ErrorCorrectedBaro::setCorrectionParams(double c, double tau) {
    correctionC = c;
    correctionTau = tau;
    refreshAlpha();
}

void ErrorCorrectedBaro::setCorrectionInputs(double motorAngleDeg, double dynamicPressureHpaIn) {
    correctionAngleDeg = motorAngleDeg;
    dynamicPressureHpa = dynamicPressureHpaIn;
}

void ErrorCorrectedBaro::setMaxCorrectionAngle(double angleDeg) {
    maxCorrectionAngleDeg = angleDeg;
}

int ErrorCorrectedBaro::init() {
    if (!innerBaro) {
        return -1;
    }
    if (!innerBaro->isInitialized()) {
        return innerBaro->begin();
    }
    return 0;
}

int ErrorCorrectedBaro::read() {
    if (!innerBaro) {
        return -1;
    }
    int err = innerBaro->update();
    if (err != 0) {
        return err;
    }
    pressure = innerBaro->getPressure();
    temp = innerBaro->getTemp();
    return 0;
}

int ErrorCorrectedBaro::update() {
    int err = Barometer::update();
    if (err != 0) {
        return err;
    }

    if (!correctionEnabled) {
        altitudeDelta = 0.0;
        return 0;
    }

    double angle = fabs(correctionAngleDeg);
    if (angle > maxCorrectionAngleDeg) {
        angle = maxCorrectionAngleDeg;
    }

    // Dynamic pressure is provided in hPa to match barometer telemetry units.
    const double qHpa = std::max(0.0, dynamicPressureHpa);
    double altitudeError = correctionC * qHpa * sin(angle * M_PI / 180.0);
    if (altitudeError > 200.0) {
        altitudeError = 200.0;
    } else if (altitudeError < -200.0) {
        altitudeError = -200.0;
    }
    altitudeDelta = (1.0 - correctionAlpha) * altitudeDelta + correctionAlpha * altitudeError;
    altitudeASL += altitudeDelta;

    return 0;
}

void ErrorCorrectedBaro::refreshAlpha() {
    const double dt = 0.1;
    correctionAlpha = dt / (dt + correctionTau);
}

} // namespace astra
