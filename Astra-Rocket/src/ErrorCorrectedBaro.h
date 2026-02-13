#ifndef ERROR_CORRECTED_BARO_H
#define ERROR_CORRECTED_BARO_H

#include <Sensors/Baro/Barometer.h>

namespace astra {

class ErrorCorrectedBaro : public Barometer {
public:
    ErrorCorrectedBaro(const char *name = "ErrorCorrectedBaro");
    ErrorCorrectedBaro(Barometer *inner, const char *name = "ErrorCorrectedBaro");

    void setInnerBarometer(Barometer *inner);

    void setCorrectionEnabled(bool enabled);
    void setCorrectionParams(double c, double tau);
    void setCorrectionInputs(double motorAngleDeg, double dynamicPressureHpa);
    void setMaxCorrectionAngle(double angleDeg);

    double getAltitudeDelta() const { return altitudeDelta; }
    double getDynamicPressure() const { return dynamicPressureHpa; }
    double getCorrectionAngle() const { return correctionAngleDeg; }

    int update(double currentTime = -1) override;

protected:
    int init() override;
    int read() override;

private:
    void refreshAlpha();

    Barometer *innerBaro = nullptr;

    bool correctionEnabled = true;
    double correctionC = 0.052;
    double correctionTau = 0.15;
    double correctionAlpha = 0.1;
    double maxCorrectionAngleDeg = 45.0;

    double correctionAngleDeg = 0.0;
    double dynamicPressureHpa = 0.0;
    double altitudeDelta = 0.0;
};

} // namespace astra

#endif // ERROR_CORRECTED_BARO_H
