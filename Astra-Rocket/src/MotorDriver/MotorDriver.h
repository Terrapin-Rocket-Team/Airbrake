#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include "Sensors/Sensor.h"

class MotorDriver : public astra::Sensor
{

protected:
    float position = 0; // current position
    float angle = 0;
    float velocity = 0; // current velocity
    float voltage = 0;

    const float kMotorMaxAngleDeg = 80.0f;
    const float kMotorPositionEpsilon = 0.02f;
    float kLimitTurnsPerSec = 100.0f;
    const float kMotorMaxPosition = 35.5f;

public:
    MotorDriver(const char *name = "MotorDriver") : Sensor(name)
    {
        addColumn("%0.3f", &position, "Motor Position");
        addColumn("%0.1f", &angle, "Motor Angle");
        addColumn("%0.3f", &velocity, "Motor Velocity");
        addColumn("%0.3f", &voltage, "Battery Voltage");
    }

    virtual float getPosition() = 0;
    virtual float getVelocity() = 0;
    virtual float getBatVoltage() = 0;
    virtual float angleToPos(float angle) = 0;
    virtual float posToAngle(float pos) = 0;

    virtual void setPos(float pos) = 0;
    virtual void setAngle(float angleDeg) = 0;

    virtual bool zeroMotor() = 0;
    virtual bool isAtLimit() = 0;

    virtual void setEnabled(bool enable) = 0;

    virtual float getMaxAngle() const { return kMotorMaxAngleDeg; }
};
// Calibrated cubic map:
// y (motor position units) = 0.127 + 0.625x - 0.0101x^2 + 9.41e-5x^3
// where x = flap angle (deg)
inline float angleToPosPoly(float angleDeg)
{
    if (angleDeg == 0.0f)
        return 0.0f; // Avoid unnecessary computation and ensure exact zero-angle position
    const float x = angleDeg;
    const float x2 = x * x;
    const float x3 = x2 * x;
    return 1.9f + (0.461f * x) - (5.92e-3f * x2) + (6.28e-5f * x3);
}
#endif // MOTOR_DRIVER_H
