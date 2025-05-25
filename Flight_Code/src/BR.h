
// BR.h
#ifndef BR_H
#define BR_H

#include <MMFS.h>
#include <USBHost_t36.h>

extern USBHost myusb;

class BR : public mmfs::Sensor {
private:
    USBHub hub1;
    
    USBSerial_BigBuffer blueRaven;
    
    static const int BUFFER_SIZE = 512;
    char buffer[BUFFER_SIZE];
    
    // Timestamp for checking connection timeout
    unsigned long lastReadTime = 0;
    static const unsigned long CONNECTION_TIMEOUT = 5000; // 5 seconds timeout
    
    bool parseMessage(const char* message);
    bool isConnected() const { return (millis() - lastReadTime) < CONNECTION_TIMEOUT; }
    void resetSensorValues();

protected:
    float altitude = 0;
    float pressure = 0;
    float temperature = 0;
    float velocity = 0; // This is not a measured value. Rather it is just the IMU velocity (intergrated accelerameter readings) but is preformed on the BlueRaven using its software
    float angle = 0;
    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;
    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;
    float rollAngle = 0;
    float tiltAngle = 0;
    float batteryVoltage = 0;
    
public:
    BR(const char *name = "BR") : 
        mmfs::Sensor("BlueRaven", name),  // Explicitly call base class constructor
        hub1(myusb), 
        blueRaven(myusb, 1) {
        setName(name);
        addColumn(mmfs::FLOAT, &batteryVoltage, "Battery Voltage (V)");
        addColumn(mmfs::FLOAT, &altitude, "ALT (m)");
        addColumn(mmfs::FLOAT, &pressure, "PRES (Pa)");
        addColumn(mmfs::FLOAT, &temperature, "TEMP (C)");
        addColumn(mmfs::FLOAT, &velocity, "VEL (m/s)");
        addColumn(mmfs::FLOAT, &angle, "ANG (deg)");
        addColumn(mmfs::FLOAT, &accelX, "ACCX (m/s^2)");
        addColumn(mmfs::FLOAT, &accelY, "ACCY (m/s^2)");
        addColumn(mmfs::FLOAT, &accelZ, "ACCZ (m/s^2)");
        addColumn(mmfs::FLOAT, &gyroX, "GYROX (rad/s)");
        addColumn(mmfs::FLOAT, &gyroY, "GYROY (rad/s)");
        addColumn(mmfs::FLOAT, &gyroZ, "GYROZ (rad/s)");
        addColumn(mmfs::FLOAT, &rollAngle, "ROLL (deg)");
        addColumn(mmfs::FLOAT, &tiltAngle, "TILT (deg)");
    }
    
    virtual ~BR() {
        if (initialized) {
            blueRaven.end();
        }
    }
    
    // Core functions
    virtual bool init() override;
    virtual void read() override;
    
    // Getters with connection check
    float getAltitude() const { return altitude; }
    float getPressure() const { return  pressure; }
    float getTemperature() const { return temperature; }
    float getTiltAngle() const { return tiltAngle; }
    float getRollAngle() const { return rollAngle; }
    float getVelocity() const { return velocity; }
    float getAccelerationX() const { return accelX; }
    float getAccelerationY() const { return accelY; }
    float getAccelerationZ() const { return  accelZ; }
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }
    
    // Status check
    bool isDeviceConnected() const { return isConnected(); }
    
    // Sensor type information
    virtual const mmfs::SensorType getType() const override { return "BlueRaven"_i; }
    virtual const char* getTypeString() const override { return "BLUE_RAVEN"; }

    int getAvailableBytes()  { return blueRaven.available(); }

};

#endif
