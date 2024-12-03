#ifndef VN_100_H
#define VN_100_H

#include <MMFS.h>
#include "vector_nav.h"

class VN_100 : public mmfs::Sensor
{
private:
    bfs::Vn100 vn;

    struct PackedData
    {
        double ax;
        double ay;
        double az;
        double ulrx;
        double ulry;
        double ulrz;
        double quatx;
        double quaty;
        double quatz;
        double quatw;
        double angvx;
        double angvy;
        double angvz;
        double magx;
        double magy;
        double magz;
        double p;
        double t;
        double dt;
        double dv_x;
        double dv_y;
        double dv_z;
        double dth_x;
        double dth_y;
        double dth_z;
    } __attribute__((packed));

public:
    VN_100(SPIClass *spiBus, const uint8_t chipSelectPin) : vn(spiBus, chipSelectPin)
    {
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
    virtual const mmfs::SensorType getType() const override { return mmfs::IMU_; } // TODO
    virtual const char *getTypeString() const override { return "VN_100"; } // TODO
    
    virtual const int getNumPackedDataPoints() const override;
    virtual const mmfs::PackedType *getPackedOrder() const override;
    virtual const char **getPackedDataLabels() const override;
    virtual void packData();

protected:
    imu::Vector<3> accelerationVec = imu::Vector<3>(0, 0, 0); // in m/s^2
    imu::Vector<3> angularVelocity = imu::Vector<3>(0, 0, 0); // in rad/s
    imu::Vector<3> orientationEuler = imu::Vector<3>(0, 0, 0); // in deg/s
    imu::Quaternion orientation = imu::Quaternion(1, 0, 0, 0);
    imu::Vector<3> magnetometer = imu::Vector<3>(0, 0, 0); // in uT
    imu::Vector<3> deltaVelocity = imu::Vector<3>(0, 0, 0); // in deg/s
    imu::Vector<3> deltaTheta = imu::Vector<3>(0, 0, 0); // in m/s^2
    double deltaTime = 0; // in s
    double pressure = 0; // in Pa
    double temperature = 0; // in deg C
};

imu::Vector<3> convertToEuler(const imu::Quaternion &orientation);
#endif