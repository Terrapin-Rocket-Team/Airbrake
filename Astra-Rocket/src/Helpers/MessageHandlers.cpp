#include "MessageHandlers.h"

#include <cstdlib>
#include <cstring>

#include "../MotorDriver/MotorDriver.h"

namespace
{
constexpr float kSweepStepDegrees = 5.0f;

struct AirbrakeSweepState
{
    bool active = false;
    float currentAngleDeg = 0.0f;
    float maxAngleDeg = 0.0f;
    uint32_t dwellMs = 0;
    uint32_t nextStepAtMs = 0;
    Stream *source = nullptr;
};

AirbrakeSweepState gSweepState;
}

static bool parseDoubleArg(const char *text, double &outValue)
{
    if (!text)
        return false;

    char *endPtr = nullptr;
    const double value = strtod(text, &endPtr);
    if (endPtr == text)
        return false;

    while (*endPtr == ' ' || *endPtr == '\t' || *endPtr == '\r' || *endPtr == '\n')
        endPtr++;

    if (*endPtr != '\0')
        return false;

    outValue = value;
    return true;
}

void handleAirbrakeMessage(const char *message, const char *prefix, Stream *source, MotorDriver &motorDriver)
{
    (void)prefix;

    if (!message || !source)
        return;

    char buffer[128];
    strncpy(buffer, message, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *command = strtok(buffer, " \t\r\n");
    if (!command)
    {
        source->println("AB ERR empty command");
        return;
    }

    if (strcmp(command, "ANGLE") == 0)
    {
        const char *arg = strtok(nullptr, " \t\r\n");
        double angleDeg = 0.0;
        if (!parseDoubleArg(arg, angleDeg))
        {
            source->println("AB ERR ANGLE requires a numeric value");
            return;
        }

        const bool sweepStopped = gSweepState.active;
        gSweepState.active = false;

        const float targetPos = motorDriver.angleToPos(static_cast<float>(angleDeg));
        motorDriver.setPos(targetPos);
        source->printf("AB OK angle=%.2f pos=%.4f sweep_stopped=%d\n",
                       angleDeg,
                       targetPos,
                       sweepStopped ? 1 : 0);
        return;
    }

    if (strcmp(command, "SWEEP") == 0)
    {
        const char *arg = strtok(nullptr, " \t\r\n");
        double dwellSeconds = 0.0;
        if (!parseDoubleArg(arg, dwellSeconds))
        {
            source->println("AB ERR SWEEP requires dwell time in seconds");
            return;
        }

        if (dwellSeconds < 0.0)
        {
            source->println("AB ERR SWEEP dwell must be >= 0 seconds");
            return;
        }

        gSweepState.active = true;
        gSweepState.currentAngleDeg = 0.0f;
        gSweepState.maxAngleDeg = motorDriver.getMaxAngle();
        gSweepState.dwellMs = static_cast<uint32_t>(dwellSeconds * 1000.0);
        gSweepState.nextStepAtMs = millis() + gSweepState.dwellMs;
        gSweepState.source = source;

        const float targetPos = motorDriver.angleToPos(gSweepState.currentAngleDeg);
        motorDriver.setPos(targetPos);

        source->printf("AB OK sweep_started step=%.1f max=%.1f dwell_s=%.3f\n",
                       kSweepStepDegrees,
                       gSweepState.maxAngleDeg,
                       dwellSeconds);

        if (gSweepState.maxAngleDeg <= 0.0f)
        {
            gSweepState.active = false;
            source->println("AB OK sweep_complete");
        }
        return;
    }

    if (strcmp(command, "SWEEP_STOP") == 0)
    {
        const bool wasActive = gSweepState.active;
        gSweepState.active = false;
        source->printf("AB OK sweep_stopped=%d\n", wasActive ? 1 : 0);
        return;
    }

    if (strcmp(command, "CRASH") == 0)
    {
        gSweepState.active = false;
        source->println("AB OK forcing crash via nullptr dereference");
        source->flush();
        delay(20);

        volatile uint32_t *crashPtr = nullptr;
        *crashPtr = 0xDEADBEEFu;

        while (true)
        {
            // Should never execute after the forced fault.
        }
        return;
    }

    if (strcmp(command, "ENABLE_MOTOR") == 0)
    {
        const char *arg = strtok(nullptr, " \t\r\n");
        if (!arg)
        {
            source->println("AB ERR ENABLE_MOTOR requires an argument (0 or 1)");
            return;
        }

        bool enable = false;
        if (strcmp(arg, "1") == 0 || strcmp(arg, "true") == 0 || strcmp(arg, "TRUE") == 0)
        {
            enable = true;
        }
        else if (strcmp(arg, "0") == 0 || strcmp(arg, "false") == 0 || strcmp(arg, "FALSE") == 0)
        {
            enable = false;
        }
        else
        {
            source->println("AB ERR ENABLE_MOTOR argument must be 0, 1, true, or false");
            return;
        }

        motorDriver.setEnabled(enable);
        source->printf("AB OK motor_enabled=%d\n", enable ? 1 : 0);
        return;
    }

    source->println("AB ERR unknown command (use ANGLE, SWEEP, SWEEP_STOP, ENABLE_MOTOR, or CRASH)");
}

void updateAirbrakeSweep(MotorDriver &motorDriver)
{
    if (!gSweepState.active)
    {
        return;
    }

    const uint32_t nowMs = millis();
    if (static_cast<int32_t>(nowMs - gSweepState.nextStepAtMs) < 0)
    {
        return;
    }

    float nextAngleDeg = gSweepState.currentAngleDeg + kSweepStepDegrees;
    if (nextAngleDeg > gSweepState.maxAngleDeg)
    {
        nextAngleDeg = gSweepState.maxAngleDeg;
    }

    gSweepState.currentAngleDeg = nextAngleDeg;
    const float targetPos = motorDriver.angleToPos(nextAngleDeg);
    motorDriver.setPos(targetPos);

    if (gSweepState.source)
    {
        gSweepState.source->printf("AB SWEEP angle=%.2f pos=%.4f\n", nextAngleDeg, targetPos);
    }

    if (gSweepState.currentAngleDeg >= gSweepState.maxAngleDeg)
    {
        gSweepState.active = false;
        if (gSweepState.source)
        {
            gSweepState.source->println("AB OK sweep_complete");
        }
        return;
    }

    gSweepState.nextStepAtMs = nowMs + gSweepState.dwellMs;
}
