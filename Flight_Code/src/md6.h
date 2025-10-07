#ifndef MD_H
#define MD_H
#include <MMFS.h>
#include <ODriveUART.h>
#include <SoftwareSerial.h>
#include "airbrake_state.h"

SoftwareSerial odrive_serial(mdrx, mdtx); // RX, TX
unsigned long baudrate = 115200;
ODriveUART odrive(odrive_serial);

namespace mmfs
{
    class MotorDriver : public mmfs::Sensor
    {
    private:
        // Add private members and methods specific to MD6 here

    protected:
        // Add protected members and methods specific to MD6 here
        double position = 0; // Example variable
        double velocity = 0; // Example variable


    public:
        MotorDriver(const char *name = "MotorDriver") : Sensor("MotorDriver", name)
        {
            setName(name);
            addColumn(mmfs::FLOAT, &position, "Motor Position");
            addColumn(mmfs::FLOAT, &velocity, "Motor Velocity");        
        }

        bool init() override;
        void read() override;
        bool isInitialized() const { return initialized; }

        float getPosition() const { return position; }
        float getVelocity() const { return velocity; }

        void setPosition(float pos);
        void setVelocity(float vel);
        void angleToPos(int angle);
    };
}


#endif // MD_H

