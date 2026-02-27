#ifndef MD_H
#define MD_H

#include "MotorDriver.h"

class MDNative : public MotorDriver
{
    uint64_t lastNativeUpdateMicros = 0;
    bool nativeTimebaseReady = false;
    double lastNativeUpdateSimTime = 0.0;
    bool nativeSimTimebaseReady = false;
    bool nativeUsingSimClock = false;
    float nativeMaxDegPerSecond = 0;
    float targetPosition = 0;
    float targetVelocity = 0;
    float targetAngle = 0;
    void updateNativeSimulation();

public:
    MDNative(const char *name = "MDNative") : MotorDriver(name)
    {
    }

    int init() override;
    int read() override;

    float getPosition();
    float getVelocity();
    float getBatVoltage();
    float angleToPos(float angle);
    float posToAngle(float pos);

    void setPos(float pos);
    void setVel(float vel);
    void setAngle(float angleDeg);

    bool zeroMotor();
    bool isAtLimit();
};

#endif // MD_H
