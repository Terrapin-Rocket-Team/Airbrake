#include "md6.h"


using namespace mmfs;
bool MotorDriver::init()
{
    odrive_serial.begin(baudrate);
    delay(1000); // Give some time for the serial connection to establish
    // Implement initialization logic for MD6 sensor

    long int timeout = millis() + 5000; // 5 second timeout
    bool deviceResponding = false;
    bool deviceResponding2 = false;

    while (millis() < timeout)
    {
        if(odrive.getState() == AXIS_STATE_UNDEFINED){
            delay(100);
        }
        else{
            deviceResponding = true;
            break;
        }
    }
    timeout += 5000; // Extend timeout by another 5 seconds
    while (deviceResponding && millis() < timeout){
        if(odrive.getState() !=AXIS_STATE_CLOSED_LOOP_CONTROL){
            odrive.clearErrors();
            odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
            delay(10);
        }
        else{
            deviceResponding2 = true;
            break;
        }
        
    }

    if (deviceResponding && deviceResponding2)
    {
        initialized = true;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Motor Driver connection successful.");
    }
    else
    {
        initialized = false;
        mmfs::getLogger().recordLogData(mmfs::ERROR_, "Motor Driver did not respond during init.");
    }

    return initialized; // Return true if initialization is successful
} 
    //gets the position and velocity from the motor driver
void MotorDriver::read()
{
    ODriveFeedback feedback = odrive.getFeedback();
    position = feedback.pos;
    velocity = feedback.vel;
}

void MotorDriver::setPosition(float pos)
{
    odrive.setPosition(pos);
}

void MotorDriver::setVelocity(float vel)
{
    odrive.setVelocity(vel);
}

void MotorDriver::angleToPos(int angle){ //need to update for actual motor
    // Convert angle in degrees to position in steps
    int position = (angle * stepGranularity) / 360;
    setPosition(position);
}


