#include "vn_100.h"
#include <RecordData/DataReporter.h>

bool VN_100::begin(bool useBiasCorrection)
{
    biasCorrectionMode = useBiasCorrection;
    return init();
}

bool VN_100::init()
{
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);
    SPI.begin();

    vn.Reset();
    if (!vn.Begin())
    {
        return initialized = false;
    }

    return initialized = true;
}

void VN_100::update()
{
    read();
}

void VN_100::read()
{
    if (vn.Read())
    {
        accelerationVec = mmfs::Vector<3>(vn.accel_x_mps2(), vn.accel_y_mps2(), vn.accel_z_mps2());
        angularVelocity = mmfs::Vector<3>(vn.gyro_x_radps(), vn.gyro_y_radps(), vn.gyro_z_radps());
        orientationEuler = mmfs::Vector<3>(vn.yaw_rad() * 180 / 3.14159, vn.pitch_rad() * 180 / 3.14159, vn.roll_rad() * 180 / 3.14159);
        magnetometer = mmfs::Vector<3>(vn.mag_x_ut(), vn.mag_y_ut(), vn.mag_z_ut());
        deltaVelocity = mmfs::Vector<3>(vn.delta_velocity_x(), vn.delta_velocity_y(), vn.delta_velocity_z());
        deltaTheta = mmfs::Vector<3>(vn.delta_theta_x(), vn.delta_theta_y(), vn.delta_theta_z());
        deltaTime = vn.delta_time();
        tilt = acos(cos(vn.pitch_rad() * cos(vn.roll_rad()))) * 180 / 3.14159;

        // TODO quaterion orientation
        pressure = vn.pres_pa();
        temperature = vn.die_temp_c();
    }
}

void VN_100::calibrate()
{
    // TODO
}

mmfs::Quaternion VN_100::getOrientation() const
{
    return orientation;
}

mmfs::Vector<3> VN_100::getAcceleration() const
{
    return accelerationVec;
}

mmfs::Vector<3> VN_100::getOrientationEuler() const
{
    return orientationEuler;
}

mmfs::Vector<3> VN_100::getMagnetometer() const
{
    return magnetometer;
}

mmfs::Vector<3> VN_100::getDeltaVelocity() const
{
    return deltaVelocity;
}

mmfs::Vector<3> VN_100::getDeltaTheta() const
{
    return deltaTheta;
}

double VN_100::getDeltaTime() const
{
    return deltaTime;
}

double VN_100::getTilt() const
{
    return tilt;
}

mmfs::Vector<3> convertToEuler(const mmfs::Quaternion &orientation)
{
    mmfs::Vector<3> euler = orientation.toEuler();
    // reverse the vector, since it returns in z, y, x
    euler = mmfs::Vector<3>(euler.x(), euler.y(), euler.z());
    return euler;
}