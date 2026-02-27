#ifndef MD_H
#define MD_H

#include <ODriveUART.h>
#include <Arduino.h>
#include "MotorDriver.h"

class MDODrive : public MotorDriver
{
private:
    ODriveFeedback feedback;

protected:
    // Add protected members and methods specific to MD6 here
    float initposition = 0; // position when zeroed
    HardwareSerial &odrive_serial;
    ODriveUART odrive;

    int topLimitSwitchPin = -1;
    int botLimitSwitchPin = -1;

    bool isLimitSwitchPressed(int pin);

    bool odrivePositionControlConfigured = false;

public:
    MDODrive(const char *name = "MotorDriver", HardwareSerial &serial = Serial2, int topLimitSwitchPin = -1, int botLimitSwitchPin = -1) : MotorDriver(name), odrive_serial(serial), odrive(serial), topLimitSwitchPin(topLimitSwitchPin), botLimitSwitchPin(botLimitSwitchPin)
    {
    }

    int init() override;
    int read() override;

    float getPosition() override;
    float getVelocity() override;
    float getBatVoltage() override;
    float angleToPos(float angle) override;
    float posToAngle(float pos) override;

    void setPos(float pos) override;
    void setAngle(float angleDeg) override;

    bool zeroMotor() override;
    bool isAtLimit() override;
};

#endif // MD_H
