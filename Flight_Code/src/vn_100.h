#ifndef VN_100_H
#define VN_100_H

#include <MMFS.h>
#include "vector_nav.h"

class VN_100 : public mmfs::Sensor
{
private:
    bfs::Vn100 vn;

public:
    VN_100(SPIClass *spiBus, const uint8_t chipSelectPin, const char *name = "VN_100") : vn(spiBus, chipSelectPin)
    {
        setName(name);
        addColumn(mmfs::FLOAT, &accelerationVec.x(), "VN-AX (m/s/s)");
        addColumn(mmfs::FLOAT, &accelerationVec.y(), "VN-AY (m/s/s)");
        addColumn(mmfs::FLOAT, &accelerationVec.z(), "VN-AZ (m/s/s)");
        addColumn(mmfs::FLOAT, &orientationEuler.x(), "VN-ULRX (deg)");
        addColumn(mmfs::FLOAT, &orientationEuler.y(), "VN-ULRY (deg)");
        addColumn(mmfs::FLOAT, &orientationEuler.z(), "VN-ULRZ (deg)");
        addColumn(mmfs::FLOAT, &angularVelocity.x(), "VN-ANGVX (rad/s)");
        addColumn(mmfs::FLOAT, &angularVelocity.y(), "VN-ANGVY (rad/s)");
        addColumn(mmfs::FLOAT, &angularVelocity.z(), "VN-ANGVZ (rad/s)");
        addColumn(mmfs::FLOAT, &magnetometer.x(), "VN-MAGX (uT)");
        addColumn(mmfs::FLOAT, &magnetometer.y(), "VN-MAGY (uT)");
        addColumn(mmfs::FLOAT, &magnetometer.z(), "VN-MAGZ (uT)");
        addColumn(mmfs::FLOAT, &pressure, "VN-P (Pa)");
        addColumn(mmfs::FLOAT, &temperature, "VN-T (C)");
        addColumn(mmfs::FLOAT, &deltaTime, "VN-DT (s)");
        addColumn(mmfs::FLOAT, &deltaVelocity.x(), "VN-DV-X (m/s/s)");
        addColumn(mmfs::FLOAT, &deltaVelocity.y(), "VN-DV-Y (m/s/s)");
        addColumn(mmfs::FLOAT, &deltaVelocity.z(), "VN-DV-Z (m/s/s)");
        addColumn(mmfs::FLOAT, &deltaTheta.x(), "VN-DTH-X (deg/s)");
        addColumn(mmfs::FLOAT, &deltaTheta.y(), "VN-DTH-Y (deg/s)");
        addColumn(mmfs::FLOAT, &deltaTheta.z(), "VN-DTH-Z (deg/s)");
    };
    virtual ~VN_100(){};
    virtual void calibrate();
    virtual bool begin(bool useBiasCorrection = true) override;
    virtual bool init() override;
    virtual void update() override;
    virtual void read() override;
    virtual imu::Quaternion getOrientation() const;
    virtual imu::Vector<3> getAcceleration() const;
    virtual imu::Vector<3> getOrientationEuler() const;
    virtual imu::Vector<3> getMagnetometer() const;
    virtual imu::Vector<3> getDeltaVelocity() const;
    virtual imu::Vector<3> getDeltaTheta() const;
    virtual double getDeltaTime() const;
    virtual const mmfs::SensorType getType() const override { return mmfs::OTHER_; } // TODO
    virtual const char *getTypeString() const override { return "VN_100"; } // TODO

protected:
    imu::Vector<3> accelerationVec = imu::Vector<3>(0, 0, 0); // in m/s^2
    imu::Vector<3> angularVelocity = imu::Vector<3>(0, 0, 0); // in rad/s
    imu::Vector<3> orientationEuler = imu::Vector<3>(0, 0, 0); // in deg/s
    imu::Quaternion orientation = imu::Quaternion(1, 0, 0, 0);
    imu::Vector<3> magnetometer = imu::Vector<3>(0, 0, 0); // in uT
    imu::Vector<3> deltaVelocity = imu::Vector<3>(0, 0, 0); // in deg/s
    imu::Vector<3> deltaTheta = imu::Vector<3>(0, 0, 0); // in m/s^2
    float deltaTime = 0; // in s
    float pressure = 0; // in Pa
    float temperature = 0; // in deg C
};

imu::Vector<3> convertToEuler(const imu::Quaternion &orientation);
#endif