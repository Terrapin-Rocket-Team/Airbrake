#include "MDNative.h"
#include <cmath>
#include <Sensors/HITL/HITLSensorBuffer.h>

void MDNative::updateNativeSimulation()
{
    float dt = 0.0f;
    bool haveSimDt = false;

    // In SITL/HITL mode, integrate against simulation timestamp so motor speed
    // is in simulation time (deg/s), independent of host execution speed.
    astra::HITLSensorBuffer &hitl = astra::HITLSensorBuffer::instance();
    const double simTime = hitl.data.timestamp;
    if (std::isfinite(simTime) && simTime >= 0.0)
    {
        if (!nativeSimTimebaseReady)
        {
            lastNativeUpdateSimTime = simTime;
            nativeSimTimebaseReady = true;
        }
        else
        {
            const double simDt = simTime - lastNativeUpdateSimTime;
            if (simDt > 0.0)
            {
                dt = static_cast<float>(simDt);
                lastNativeUpdateSimTime = simTime;
                haveSimDt = true;
                nativeUsingSimClock = true;
            }
            else if (nativeUsingSimClock)
            {
                // Multiple reads can happen in one FC cycle; only integrate once
                // for each new simulation timestamp.
                return;
            }
        }
    }

    if (!haveSimDt)
    {
        const uint64_t nowMicros = micros();
        if (!nativeTimebaseReady)
        {
            lastNativeUpdateMicros = nowMicros;
            nativeTimebaseReady = true;
            return;
        }

        dt = static_cast<float>(nowMicros - lastNativeUpdateMicros) / 1000000.0f;
        lastNativeUpdateMicros = nowMicros;
    }

    if (dt <= 0.0f)
    {
        return;
    }

    // Integrate in smaller chunks so simulated speed remains stable even if
    // read() calls are sporadic.
    const float startPos = position;
    float remaining = dt;
    while (remaining > 0.0f)
    {
        const float chunkDt = (remaining > 0.02f) ? 0.02f : remaining;
        const float currentAngle = posToAngle(position);
        const float desiredAngle = posToAngle(targetPosition);
        const float maxAngleStep = nativeMaxDegPerSecond * chunkDt;

        float angleStep = desiredAngle - currentAngle;
        if (angleStep > maxAngleStep)
            angleStep = maxAngleStep;
        else if (angleStep < -maxAngleStep)
            angleStep = -maxAngleStep;

        position = angleToPos(currentAngle + angleStep);

        if (position < 0.0f)
            position = 0.0f;
        else if (position > kMotorMaxPosition)
            position = kMotorMaxPosition;

        if (std::fabs(targetPosition - position) < 1e-4f)
        {
            position = targetPosition;
            break;
        }

        remaining -= chunkDt;
    }

    velocity = (position - startPos) / dt;
    angle = posToAngle(position);
    targetAngle = posToAngle(targetPosition);

    // Simple battery sag model under load for more realistic telemetry.
    voltage = 16.0f - (0.01f * std::fabs(velocity));
    if (voltage < 13.5f)
        voltage = 13.5f;
}

int MDNative::init()
{
    LOGI("Initializing simulated motor driver (native)");
    initialized = true;
    position = 0;
    velocity = 0;
    angle = 0;
    targetPosition = 0;
    targetVelocity = 0;
    targetAngle = 0;
    nativeMaxDegPerSecond = kLimitTurnsPerSec * 360.0f; // convert from turns/s to deg/s
    lastNativeUpdateMicros = 0;
    nativeTimebaseReady = false;
    lastNativeUpdateSimTime = 0.0;
    nativeSimTimebaseReady = false;
    nativeUsingSimClock = false;
    voltage = 16.0f;
    LOGI("Motor simulator max flap speed: %0.1f deg/s", nativeMaxDegPerSecond);
    return 0;
}
// gets the position and velocity from the motor driver
int MDNative::read()
{
    updateNativeSimulation();
    return 0;
}

float MDNative::getPosition() // since last read() relative to initposition;
{
    return position;
}

float MDNative::getVelocity() // since last read()
{
    return velocity;
}

void MDNative::setPos(float pos)
{
    // Clamp to travel range. Float rounding can produce tiny overshoot at limits
    // (e.g., angleToPos(90) -> 26.000002f), which should still map to full deploy.
    if (pos < 0.0f)
    {
        pos = 0.0f;
    }
    else if (pos > kMotorMaxPosition)
    {
        pos = kMotorMaxPosition;
    }
    targetAngle = posToAngle(pos);
    updateNativeSimulation();
    targetPosition = pos;
    targetVelocity = 0;
    return;
}

void MDNative::setVel(float vel) // TODO: make sure directions are correct
{
    updateNativeSimulation();
    targetVelocity = vel;
    targetPosition += vel * 0.02f;
    if (targetPosition < 0.0f)
        targetPosition = 0.0f;
    else if (targetPosition > kMotorMaxPosition)
        targetPosition = kMotorMaxPosition;
    targetAngle = posToAngle(targetPosition);
    return;
}

float MDNative::angleToPos(float angle)
{
    // Convert flap angle in degrees to motor position units using calibrated cubic.
    if (angle < 0.0f)
        angle = 0.0f;
    else if (angle > kMotorMaxAngleDeg)
        angle = kMotorMaxAngleDeg;

    float pos = angleToPosPoly(angle);
    if (pos < 0.0f)
        pos = 0.0f;
    else if (pos > kMotorMaxPosition)
        pos = kMotorMaxPosition;
    return pos;
}

float MDNative::posToAngle(float pos)
{
    // Invert calibrated cubic with monotonic bisection on [0, kMotorMaxAngleDeg].
    if (pos < 0.0f)
        pos = 0.0f;
    else if (pos > kMotorMaxPosition)
        pos = kMotorMaxPosition;

    float low = 0.0f;
    float high = kMotorMaxAngleDeg;
    for (int i = 0; i < 24; i++)
    {
        const float mid = 0.5f * (low + high);
        const float pMid = angleToPosPoly(mid);
        if (pMid < pos)
            low = mid;
        else
            high = mid;
    }
    return 0.5f * (low + high);
}

bool MDNative::zeroMotor()
{
    position = 0;
    targetPosition = 0;
    targetVelocity = 0;
    angle = 0;
    targetAngle = 0;
    velocity = 0;
    lastNativeUpdateMicros = 0;
    nativeTimebaseReady = false;
    lastNativeUpdateSimTime = 0.0;
    nativeSimTimebaseReady = false;
    nativeUsingSimClock = false;
    LOGI("Simulated motor zeroed successfully.");
    return true;
}

bool MDNative::isAtLimit()
{
    return (position <= 0.0f) || (position >= kMotorMaxPosition);
}

float MDNative::getBatVoltage()
{
    return voltage;
}

void MDNative::setAngle(float angleDeg)
{
    const float pos = angleToPos(angleDeg);
    setPos(pos);
}