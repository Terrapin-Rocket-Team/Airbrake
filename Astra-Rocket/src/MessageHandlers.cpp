#include "MessageHandlers.h"
#include <cstdlib>
#include <cstring>
#include "AirbrakeController.h"
#include "MotorDriver/MotorDriver.h"
#include "AirbrakeController.h"

static constexpr double kDefaultMinAngleDeg = 0.0;
static constexpr double kDefaultMaxAngleDeg = 73.0;

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

void handleAirbrakeMessage(const char *message, const char *prefix, Stream *source,
                          AirbrakeController &airbrakeCtrl, MotorDriver &motorDriver)
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

    if (strcmp(command, "TARGET_APOGEE") == 0)
    {
        if (!airbrakeCtrl)
        {
            source->println("AB ERR controller not ready");
            return;
        }
        const char *arg = strtok(nullptr, " \t\r\n");
        double apogeeM = 0.0;
        if (!parseDoubleArg(arg, apogeeM))
        {
            source->println("AB ERR TARGET_APOGEE requires a numeric value");
            return;
        }

        airbrakeCtrl.setTargetApogee(apogeeM);
        source->printf("AB OK target_apogee=%.2f\n", apogeeM);
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

        if (angleDeg < kDefaultMinAngleDeg)
            angleDeg = kDefaultMinAngleDeg;
        else if (angleDeg > kDefaultMaxAngleDeg)
            angleDeg = kDefaultMaxAngleDeg;

        const float targetPos = motorDriver.angleToPos(static_cast<float>(angleDeg));
        motorDriver.setPos(targetPos);
        source->printf("AB OK angle=%.2f pos=%.4f\n", angleDeg, targetPos);
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

    source->println("AB ERR unknown command (use TARGET_APOGEE, ANGLE, or ENABLE_MOTOR)");
}

void handleHitlMessage(const char *message, const char *prefix, Stream *source, bool hitlRuntimeReady)
{
    (void)prefix;
    if (!message || !source)
        return;

    char buffer[64];
    strncpy(buffer, message, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *command = strtok(buffer, " \t\r\n");
    if (!command)
    {
        source->println("HITL WAIT");
        return;
    }

    if ((strcmp(command, "READY?") == 0) || (strcmp(command, "PING") == 0))
    {
        source->println(hitlRuntimeReady ? "HITL READY" : "HITL WAIT");
        return;
    }

    source->println("HITL WAIT");
}
