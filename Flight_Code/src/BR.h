
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
    
    struct PackedData {
        float altitude;    // in feet
        float pressure;    // in Pa
        float temperature; // in C
        float velocity;    // in m/s
        float angle;      // in degrees
        float accelX;     // in m/s^2
        float accelY;     // in m/s^2
        float accelZ;     // in m/s^2
        float gyroX;      // in rad/s
        float gyroY;      // in rad/s
        float gyroZ;      // in rad/s
        float rollAngle;  // in degrees
        float tiltAngle;  // in degrees
    } __attribute__((packed));
    
    bool parseMessage(const char* message);
    bool isConnected() const { return (millis() - lastReadTime) < CONNECTION_TIMEOUT; }
    void resetSensorValues();

protected:
    float altitude = 0;
    float pressure = 0;
    float temperature = 0;
    float velocity = 0;
    float angle = 0;
    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;
    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;
    float rollAngle = 0;
    float tiltAngle = 0;
    
public:
    BR(const char *name = "BR") : 
        mmfs::Sensor(),  // Explicitly call base class constructor
        hub1(myusb), 
        blueRaven(myusb, 1) {
        setName(name);
        addColumn(mmfs::FLOAT, &altitude, "BR-ALT (ft)");
        addColumn(mmfs::FLOAT, &pressure, "BR-PRES (Pa)");
        addColumn(mmfs::FLOAT, &temperature, "BR-TEMP (C)");
        addColumn(mmfs::FLOAT, &velocity, "BR-VEL (m/s)");
        addColumn(mmfs::FLOAT, &angle, "BR-ANG (deg)");
        addColumn(mmfs::FLOAT, &accelX, "BR-ACCX (m/s^2)");
        addColumn(mmfs::FLOAT, &accelY, "BR-ACCY (m/s^2)");
        addColumn(mmfs::FLOAT, &accelZ, "BR-ACCZ (m/s^2)");
        addColumn(mmfs::FLOAT, &gyroX, "BR-GYROX (rad/s)");
        addColumn(mmfs::FLOAT, &gyroY, "BR-GYROY (rad/s)");
        addColumn(mmfs::FLOAT, &gyroZ, "BR-GYROZ (rad/s)");
        addColumn(mmfs::FLOAT, &rollAngle, "BR-ROLL (deg)");
        addColumn(mmfs::FLOAT, &tiltAngle, "BR-TILT (deg)");
    }
    
    virtual ~BR() {
        if (initialized) {
            blueRaven.end();
        }
    }
    
    // Core functions
    virtual bool begin(bool useBiasCorrection = true) override;
    virtual bool init() override;
    virtual void update() override;
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
    virtual const mmfs::SensorType getType() const override { return mmfs::OTHER_; }
    virtual const char* getTypeString() const override { return "BLUE_RAVEN"; }

    int getAvailableBytes()  { return blueRaven.available(); }

};

#endif
