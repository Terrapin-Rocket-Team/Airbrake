#include "MDODrive.h"
#include <cmath>

int MDODrive::init()
{

    LOGI("Initializing Motor Driver...");
    odrive_serial.begin(115200);
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
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(500);
    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL)
    {
        LOGI("still enabling...");
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(1000);
    }

    LOGI("ODrive running!");

    // Configure control/input modes once at init to avoid repeated String
    // allocations and serial parameter writes in the high-rate setPos path.
    odrive.setParameter("axis0.controller.config.vel_limit", String(kLimitTurnsPerSec, 3));
    odrive.setParameter("axis0.controller.config.control_mode", String((long)CONTROL_MODE_POSITION_CONTROL));
    odrive.setParameter("axis0.controller.config.input_mode", String((long)INPUT_MODE_PASSTHROUGH));
    LOGI("ODrive vel_limit (turn/s): %0.2f", odrive.getParameterAsFloat("axis0.controller.config.vel_limit"));
    odrivePositionControlConfigured = true;

    pinMode(topLimitSwitchPin, INPUT_PULLUP);
    pinMode(botLimitSwitchPin, INPUT_PULLUP);
    initialized = true;
    return 0;
}
// gets the position and velocity from the motor driver
int MDODrive::read()
{

    feedback = odrive.getFeedback();
    position = initposition - feedback.pos;
    velocity = feedback.vel;
    voltage = odrive.getParameterAsFloat("vbus_voltage");
    angle = posToAngle(position);

    //check every loop if limit switch is pressed. If so, stop motor.
    if (isLimitSwitchPressed(topLimitSwitchPin))
    {
        LOGW("Top limit switch pressed, stopping motor at position %0.2f (%0.2f deg)", position, angle);
        odrive.setVelocity(0);
    }
    else if (isLimitSwitchPressed(botLimitSwitchPin))
    {
        LOGW("Bottom limit switch pressed, stopping motor at position %0.2f (%0.2f deg)", position, angle);
        odrive.setVelocity(0);
    }

    return 0;
}

float MDODrive::getPosition() // since last read() relative to initposition;
{
    return position;
}

float MDODrive::getVelocity() // since last read()
{
    return velocity;
}

void MDODrive::setPos(float pos)
{
    setEnabled(true);
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
    if (!odrivePositionControlConfigured)
    {
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrive.setParameter("axis0.controller.config.control_mode", String((long)CONTROL_MODE_POSITION_CONTROL));
        odrive.setParameter("axis0.controller.config.input_mode", String((long)INPUT_MODE_PASSTHROUGH));
        odrivePositionControlConfigured = true;
    }
    const bool atTarget = std::fabs(pos - position) <= kMotorPositionEpsilon;
    if (atTarget)
    {
        return;
    }

    if (isLimitSwitchPressed(topLimitSwitchPin) && pos <= position)
    {
        LOGW("Top limit switch pressed, not moving motor");
        return;
    }
    else if (isLimitSwitchPressed(botLimitSwitchPin) && pos >= position)
    {
        LOGW("Bottom limit switch pressed, not moving motor");
        return;
    }

    odrive.setPosition(initposition - pos);
}

float MDODrive::angleToPos(float angle)
{
    // Convert flap angle in degrees to motor position units using calibrated cubic.
    return angleToPosPoly(angle);
}

float MDODrive::posToAngle(float pos)
{
    // Invert calibrated cubic with monotonic bisection on [0, kMotorMaxAngleDeg].
    float low = 0.0f;
    float high = kMotorMaxAngleDeg;
    for (int i = 0; i < 10; i++)
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

bool MDODrive::zeroMotor()
{
    setEnabled(true);
    if (!initialized)
    {
        LOGE("Motor Driver not initialized. Cannot zero motor.");
        return false;
    }
    // Move the motor towards the top limit switch until it is triggered
    LOGI("Zeroing motor...");
    odrive.setVelocity(0); // Move up at a constant speed
    read();

    while (isLimitSwitchPressed(topLimitSwitchPin))
    {
        read();
        odrive.setVelocity(1); // Move up at a constant speed
        delay(50);            // Small delay to allow movement
    }

    odrive.setVelocity(0); // Stop the motor
    delay(500);            // Wait for a moment to ensure the motor has stopped
    read();

    initposition = -getPosition(); // Update internal position variable
    // odrive.setState(AXIS_STATE_IDLE); turns off constant power usage, but must be undone before it can move
    LOGI("Motor zeroed successfully.");
    return true;
}

bool MDODrive::isLimitSwitchPressed(int pin)
{
    return digitalRead(pin) == HIGH;
}

bool MDODrive::isAtLimit()
{
    return isLimitSwitchPressed(topLimitSwitchPin) || isLimitSwitchPressed(botLimitSwitchPin);
}

void MDODrive::setAngle(float angleDeg)
{
    setPos(angleToPos(angleDeg));
}

float MDODrive::getBatVoltage()
{
    return voltage;
}

void MDODrive::setEnabled(bool enable)
{
    if(enable == motorEnabled)
    {
        return; // No change
    }
    if (enable)
    {
        LOGI("Enabling motor...");
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        setPos(position); // Hold current position rather than moving somewhere unwanted
        motorEnabled = true;
    }
    else
    {
        LOGI("Disabling motor...");
        odrive.setVelocity(0); //Stops all motion commands
        odrive.setState(AXIS_STATE_IDLE);
        motorEnabled = false;
    }
}