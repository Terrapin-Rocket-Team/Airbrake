#include "md6.h"
#include <cmath>

#if defined(NATIVE)
#include <Sensors/HITL/HITLSensorBuffer.h>
#ifndef MOTOR_SIM_MAX_DEG_PER_SEC
#define MOTOR_SIM_MAX_DEG_PER_SEC 50.0f
#endif
#endif

#if !defined(NATIVE)
HardwareSerial &odrive_serial = Serial1;
unsigned long baudrate = 115200;
ODriveUART odrive(odrive_serial);
#endif

using namespace astra;

namespace
{
constexpr float kMotorMaxPosition = 26.0f;
constexpr float kMotorMaxAngleDeg = 85.0f;
}

#if defined(NATIVE)
void MotorDriver::updateNativeSimulation()
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
    float remaining = dt;
    float totalStep = 0.0f;
    while (remaining > 0.0f)
    {
        const float chunkDt = (remaining > 0.02f) ? 0.02f : remaining;
        const float positionError = targetposition - position;
        const float maxStep = nativeMaxPosPerSecond * chunkDt;

        float step = positionError;
        if (step > maxStep)
            step = maxStep;
        else if (step < -maxStep)
            step = -maxStep;

        position += step;
        totalStep += step;

        if (position < 0.0f)
            position = 0.0f;
        else if (position > kMotorMaxPosition)
            position = kMotorMaxPosition;

        if (std::fabs(targetposition - position) < 1e-4f)
        {
            position = targetposition;
            break;
        }

        remaining -= chunkDt;
    }

    velocity = totalStep / dt;
    angle = posToAngle(position);
    targetAngle = posToAngle(targetposition);

    // Simple battery sag model under load for more realistic telemetry.
    voltage = 16.0f - (0.01f * std::fabs(velocity));
    if (voltage < 13.5f)
        voltage = 13.5f;
}
#endif

int MotorDriver::init()
{
#if defined(NATIVE)
    LOGI("Initializing simulated motor driver (native)");
    initialized = true;
    position = 0;
    velocity = 0;
    angle = 0;
    initposition = 0;
    targetposition = 0;
    targetvelocity = 0;
    targetAngle = 0;
    nativeMaxPosPerSecond = angleToPos(MOTOR_SIM_MAX_DEG_PER_SEC);
    lastNativeUpdateMicros = 0;
    nativeTimebaseReady = false;
    lastNativeUpdateSimTime = 0.0;
    nativeSimTimebaseReady = false;
    nativeUsingSimClock = false;
    voltage = 16.0f;
    LOGI("Motor simulator max flap speed: %0.1f deg/s", MOTOR_SIM_MAX_DEG_PER_SEC);
    return 0;
#else
    LOGI("Initializing Motor Driver...");
    odrive_serial.begin(baudrate);
    delay(1000); // Give some time for the serial connection to establish
    // Implement initialization logic for MD6 sensor

    LOGI("Waiting for ODrive...");
    while (odrive.getState() == AXIS_STATE_UNDEFINED)
    {
        delay(100);
    }

    LOGI("found ODrive");

    LOGI("ODrive Voltage: %0.2f", odrive.getParameterAsFloat("vbus_voltage"));

    LOGI("Enabling closed loop control...");
    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL)
    {
        LOGI("still enabling...");
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(1000);
    }

    LOGI("ODrive running!");

    pinMode(topLimitSwitchPin, INPUT_PULLUP);
    initialized = true;
    return 0;
#endif
}
// gets the position and velocity from the motor driver
int MotorDriver::read()
{
#if defined(NATIVE)
    updateNativeSimulation();
    return 0;
#else
    feedback = odrive.getFeedback();
    position = initposition - feedback.pos;
    velocity = feedback.vel;
    voltage = odrive.getParameterAsFloat("vbus_voltage");
    angle = posToAngle(position);
    return 0;
#endif
}

float MotorDriver::getPosition() // since last read() relative to initposition;
{
#if defined(NATIVE)
    updateNativeSimulation();
    return position;
#else
    position = initposition - feedback.pos;
    return position;
#endif
}

float MotorDriver::getVelocity() // since last read()
{
#if defined(NATIVE)
    updateNativeSimulation();
    return velocity;
#else
    velocity = feedback.vel;
    return velocity;
#endif
}

void MotorDriver::setPos(float pos)
{
    // Clamp to travel range. Float rounding can produce tiny overshoot at limits
    // (e.g., angleToPos(85) -> 26.000002f), which should still map to full deploy.
    if (pos < 0.0f)
    {
        pos = 0.0f;
    }
    else if (pos > kMotorMaxPosition)
    {
        pos = kMotorMaxPosition;
    }
    targetAngle = posToAngle(pos);
#if defined(NATIVE)
    updateNativeSimulation();
    targetposition = pos;
    targetvelocity = 0;
    return;
#else
    targetposition = initposition - pos;
    if (!motorStall())
    {
        odrive.setPosition(targetposition);
    }
    else if ((stalledstate == TOP || stalledstate == STOPPED) && pos > position)
    {
        odrive.setPosition(targetposition);
    }
    else if (stalledstate == BOTTOM && pos < position)
    {
        odrive.setPosition(targetposition);
    }
    else
    {
        LOGI("cant move");
        LOGI("Stalled State: %s", String(stalledstate).c_str());
        LOGI("Current Position: %s", String(position).c_str());
        LOGI("Target Position: %s", String(pos).c_str());
    }
#endif
}

void MotorDriver::setVel(float vel) // TODO: make sure directions are correct
{
#if defined(NATIVE)
    updateNativeSimulation();
    targetvelocity = vel;
    targetposition += vel * 0.02f;
    if (targetposition < 0.0f)
        targetposition = 0.0f;
    else if (targetposition > kMotorMaxPosition)
        targetposition = kMotorMaxPosition;
    targetAngle = posToAngle(targetposition);
    return;
#else
    if (!motorStall())
    {
        targetvelocity = vel;
    }
    else if (stalledstate == TOP && vel < 0)
    {
        targetvelocity = vel;
    }
    else if (stalledstate == BOTTOM && vel > 0)
    {
        targetvelocity = vel;
    }
    else
    {
        targetvelocity = 0;
    }
#endif
}

float MotorDriver::angleToPos(float angle)
{
    // Convert flap angle in degrees to motor position units.
    float pos = (kMotorMaxPosition / kMotorMaxAngleDeg) * angle;
    return pos;
}

float MotorDriver::posToAngle(float pos)
{
    // Convert motor position units back to flap angle in degrees.
    float angle = (pos * kMotorMaxAngleDeg / kMotorMaxPosition);
    return angle;
}

bool MotorDriver::zeroMotor()
{
    if (!initialized)
    {
        LOGE("Motor Driver not initialized. Cannot zero motor.");
        return false;
    }

#if defined(NATIVE)
    initposition = 0;
    position = 0;
    targetposition = 0;
    targetvelocity = 0;
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
#else
    // Move the motor towards the bottom limit switch until it is triggered
    LOGI("Zeroing motor...");
    odrive.setVelocity(0); // Move up at a constant speed

    while (!motorStall()) // Assuming HIGH means not triggered
    {
        read();
        odrive.setVelocity(1); // Move down at a constant speed
        delay(100);            // Small delay to allow movement
    }

    odrive.setVelocity(0); // Stop the motor
    delay(500);            // Wait for a moment to ensure the motor has stopped

    initposition = -getPosition(); // Update internal position variable
    // odrive.setState(AXIS_STATE_IDLE); turns off constant power usage, but must be undone before it can move
    LOGI("Motor zeroed successfully.");
    return true;
#endif
}

bool MotorDriver::motorStall() // TODO: make sure directions are correct
{
#if defined(NATIVE)
    return false;
#else
    bool stalled = false;

    // Read limit switches
    bool topLimitSwitchState = digitalRead(topLimitSwitchPin) == LOW;
    // position doesnt change
    for (int i = 0; i < motorstallcounter; i++)
    {
        if (positionHistory[i] != position)
        {
            stalledstate = MOVING;
            stalled = false;
            break;
        }
        else
        {
            stalled = true;
        }
    }

    if (!stalled)
    {
        if (topLimitSwitchState)
        {
            stalledstate = TOP;
            stalled = true;
        }
        else
        {
            stalledstate = MOVING;
            stalled = false;
        }
    }
    else
    {
        stalledstate = STOPPED;
    }

    return stalled;
#endif
}
