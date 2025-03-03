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
        addColumn(mmfs::DOUBLE, accelerationVec.xp, "VN-AX (m/s/s)");
        addColumn(mmfs::DOUBLE, accelerationVec.yp, "VN-AY (m/s/s)");
        addColumn(mmfs::DOUBLE, accelerationVec.zp, "VN-AZ (m/s/s)");
        addColumn(mmfs::DOUBLE, orientationEuler.xp, "VN-ULRX (deg)");
        addColumn(mmfs::DOUBLE, orientationEuler.yp, "VN-ULRY (deg)");
        addColumn(mmfs::DOUBLE, orientationEuler.zp, "VN-ULRZ (deg)");
        addColumn(mmfs::DOUBLE, angularVelocity.xp, "VN-ANGVX (rad/s)");
        addColumn(mmfs::DOUBLE, angularVelocity.yp, "VN-ANGVY (rad/s)");
        addColumn(mmfs::DOUBLE, angularVelocity.zp, "VN-ANGVZ (rad/s)");
        addColumn(mmfs::DOUBLE, magnetometer.xp, "VN-MAGX (uT)");
        addColumn(mmfs::DOUBLE, magnetometer.yp, "VN-MAGY (uT)");
        addColumn(mmfs::DOUBLE, magnetometer.zp, "VN-MAGZ (uT)");
        addColumn(mmfs::FLOAT, &pressure, "VN-P (Pa)");
        addColumn(mmfs::FLOAT, &temperature, "VN-T (C)");
        addColumn(mmfs::FLOAT, &deltaTime, "VN-DT (s)");
        addColumn(mmfs::DOUBLE, deltaVelocity.xp, "VN-DV-X (m/s/s)");
        addColumn(mmfs::DOUBLE, deltaVelocity.yp, "VN-DV-Y (m/s/s)");
        addColumn(mmfs::DOUBLE, deltaVelocity.zp, "VN-DV-Z (m/s/s)");
        addColumn(mmfs::DOUBLE, deltaTheta.xp, "VN-DTH-X (deg/s)");
        addColumn(mmfs::DOUBLE, deltaTheta.yp, "VN-DTH-Y (deg/s)");
        addColumn(mmfs::DOUBLE, deltaTheta.zp, "VN-DTH-Z (deg/s)");
    };
    virtual ~VN_100(){};
    virtual void calibrate();
    virtual bool begin(bool useBiasCorrection = true) override;
    virtual bool init() override;
    virtual void update() override;
    virtual void read() override;
    virtual mmfs::Quaternion getOrientation() const;
    virtual mmfs::Vector<3> getAcceleration() const;
    virtual mmfs::Vector<3> getOrientationEuler() const;
    virtual mmfs::Vector<3> getMagnetometer() const;
    virtual mmfs::Vector<3> getDeltaVelocity() const;
    virtual mmfs::Vector<3> getDeltaTheta() const;
    virtual double getDeltaTime() const;
    virtual const mmfs::SensorType getType() const override { return mmfs::OTHER_; } // TODO
    virtual const char *getTypeString() const override { return "VN_100"; } // TODO

protected:
    mmfs::Vector<3> accelerationVec = mmfs::Vector<3>(0, 0, 0); // in m/s^2
    mmfs::Vector<3> angularVelocity = mmfs::Vector<3>(0, 0, 0); // in rad/s
    mmfs::Vector<3> orientationEuler = mmfs::Vector<3>(0, 0, 0); // in deg/s
    mmfs::Quaternion orientation = mmfs::Quaternion(1, 0, 0, 0);
    mmfs::Vector<3> magnetometer = mmfs::Vector<3>(0, 0, 0); // in uT
    mmfs::Vector<3> deltaVelocity = mmfs::Vector<3>(0, 0, 0); // in deg/s
    mmfs::Vector<3> deltaTheta = mmfs::Vector<3>(0, 0, 0); // in m/s^2
    float deltaTime = 0; // in s
    float pressure = 0; // in Pa
    float temperature = 0; // in deg C
};

mmfs::Vector<3> convertToEuler(const mmfs::Quaternion &orientation);
#endif