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
    
    struct PackedData {
        float altitude;     // in feet
        float pressure;     // in Pa
        float temperature;  // in C
        float velocity;     // in m/s
        float angle;        // in degrees
        float acceleration; // in m/s^2
        float gyro;        // in rad/s
    } __attribute__((packed));
    
    bool parseMessage(const char* message);

protected:
    float altitude = 0;      // in feet
    float pressure = 0;      // in Pa
    float temperature = 0;   // in C
    float velocity = 0;      // in m/s
    float angle = 0;        // in degrees
    float acceleration = 0;  // in m/s^2
    float gyro = 0;         // in rad/s

public:
    BR(const char *name = "BR") : 
        mmfs::Sensor(),  // Add base class constructor
        hub1(myusb), 
        blueRaven(myusb, 1) 
    {
        setName(name);
        setUpPackedData();
    }
    
    virtual ~BR() {}

    // Core functions
    virtual bool begin(bool useBiasCorrection = true) override;
    virtual bool init() override;
    virtual void update() override;
    virtual void read() override;

    // Getters
    float getAltitude() const { return altitude; }
    float getPressure() const { return pressure; }
    float getTemperature() const { return temperature; }
    float getVelocity() const { return velocity; }
    float getAngle() const { return angle; }
    float getAcceleration() const { return acceleration; }
    float getGyro() const { return gyro; }

    // Sensor type information
    virtual const mmfs::SensorType getType() const override { return mmfs::OTHER_; }
    virtual const char* getTypeString() const override { return "BLUE_RAVEN"; }

    // Data packing functions
    virtual const int getNumPackedDataPoints() const override;
    virtual const mmfs::PackedType* getPackedOrder() const override;
    virtual const char** getPackedDataLabels() const override;
    virtual void packData() override;  // Add override specifier
};

#endif
