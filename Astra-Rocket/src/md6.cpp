#include "md6.h"

HardwareSerial &odrive_serial = Serial1;
unsigned long baudrate = 115200;
ODriveUART odrive(odrive_serial);

using namespace astra;
bool MotorDriver::init()
{
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
    return true;
}
// gets the position and velocity from the motor driver
bool MotorDriver::read()
{
    feedback = odrive.getFeedback();
    position = initposition - feedback.pos;
    velocity = feedback.vel;
    voltage = odrive.getParameterAsFloat("vbus_voltage");
    angle = posToAngle(position);
    return true;
}

float MotorDriver::getPosition() // since last read() relative to initposition;
{
    position = initposition - feedback.pos;
    return position;
}

float MotorDriver::getVelocity() // since last read()
{
    velocity = feedback.vel;
    return velocity;
}

void MotorDriver::setPos(float pos)
{
    if (pos < 0 || pos > 26)
    { // limit to 0-65 degrees
        LOGI("Position out of bounds");
        return;
    }
    if (!motorEnabled) //if the motor is not enabled, gives error warning because input will not be read
    {
        LOGI("Motor disabled. Ignoring position command.")
        return;
    }
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
}

void MotorDriver::setVel(float vel) // TODO: make sure directions are correct
{
    if (!motorEnabled) //if the motor is not enabled, gives error warning because input will not be read
    {
        LOGI("Motor disabled. Ignoring velocity command.")
        return;
    }
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
}

float MotorDriver::angleToPos(float angle)
{
    // Convert angle in degrees to position in steps
    float pos = 26 / 80.0 * angle; // 80 steps per degree
    return pos;
}

float MotorDriver::posToAngle(float pos)
{
    // Convert position in steps to angle in degrees
    float angle = (pos * 80 / 26.0);
    return angle;
}

bool MotorDriver::zeroMotor()
{
    if (!initialized)
    {
        LOGE("Motor Driver not initialized. Cannot zero motor.");
        return false;
    }

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
}

bool MotorDriver::motorStall() // TODO: make sure directions are correct
{
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
}

void MotorDriver::enableMotor() //function to enable motor 
{
    if (!initilized)
    {
        LOGE("Motor not initialized.");
        return;
    }

    LOGI("Enabling motor (closed loop control)...");

    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);

    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL)
    {
        delay(100);
    }

    read(); // get actual motor position

    // Hold current position instead of moving somewhere random
    odrive.setPosition(initposition - feedback.pos);

    motorEnabled=true;
}

void MotorDriver::disableMotor() //function to disable the motor
{
    if(!initialized)
    {
        LOGE("Motor not initialized.");
        return;
    }

    LOGI("Disabling motor (idle)...");

    odrive.setVelocity(0); //Stops all motion commands
    odrive.setState(AXIS_STATE_IDLE);

    motorEnabled = false;
}