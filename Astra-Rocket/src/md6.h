#ifndef MD_H
#define MD_H
#include <ODriveUART.h>
#include <SoftwareSerial.h>
#include "Utils/CircBuffer.h"
#include "Sensors/Sensor.h"


enum StalledState{
    STOPPED,
    TOP,
    BOTTOM,
    MOVING,
    ERROR
};

namespace astra
{
    class MotorDriver : public Sensor
    {
    private:
        int motorstallcounter = 10;
        ODriveFeedback feedback;
        CircBuffer<float> positionHistory = CircBuffer<float>(motorstallcounter);
               

    protected:
        // Add protected members and methods specific to MD6 here
        float position = 0; // current position
        float velocity = 0; // current velocity
        float initposition = 0; // position when zeroed
        float targetposition = 0; // target position
        float targetvelocity = 0; // target velocity

        int topLimitSwitchPin = 35;
        StalledState stalledstate = STOPPED;
        

    public:
        MotorDriver(const char *name = "MotorDriver") : Sensor("MotorDriver", name)
        {
            setName(name);
            addColumn("%0.3f", &position, "Motor Position");
            addColumn("%0.3f", &velocity, "Motor Velocity");
            pinMode(topLimitSwitchPin, INPUT);
        }

        bool init() override;
        bool read() override;
        bool isInitialized() const { return initialized; }

        float getPosition();
        float getVelocity();
        float angleToPos(int angle);
        float posToAngle(float pos);

        void setPos(float pos);
        void setVel(float vel);

        bool zeroMotor();
        bool motorStall();
    };
}


#endif // MD_H
