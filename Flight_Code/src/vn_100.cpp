#include "vn_100.h"

bool VN_100::begin(bool useBiasCorrection)
{
    biasCorrectionMode = useBiasCorrection;
    return init();
}

bool VN_100::init()
{
    
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
    if (vn.Read()) {
        accelerationVec = imu::Vector<3>(vn.accel_x_mps2(), vn.accel_y_mps2(), vn.accel_z_mps2());
        angularVelocity = imu::Vector<3>(vn.gyro_x_radps(), vn.gyro_y_radps(), vn.gyro_z_radps());
        orientationEuler = imu::Vector<3>(vn.yaw_rad() * 180 / 3.14159, vn.pitch_rad() * 180 / 3.14159, vn.roll_rad() * 180 / 3.14159);
        magnetometer = imu::Vector<3>(vn.mag_x_ut(), vn.mag_y_ut(), vn.mag_z_ut());
        // TODO quaterion orientation
        pressure = vn.pres_pa();
        temperature = vn.die_temp_c();
    }
}

void VN_100::calibrate()
{
    // TODO
}

imu::Quaternion VN_100::getOrientation()
{
    return orientation;
}

imu::Vector<3> VN_100::getAcceleration()
{
    return accelerationVec;
}

imu::Vector<3> VN_100::getOrientationEuler()
{
    return orientationEuler;
}

imu::Vector<3> VN_100::getMagnetometer()
{
    return magnetometer;
}

imu::Vector<3> convertToEuler(const imu::Quaternion &orientation)
{
    imu::Vector<3> euler = orientation.toEuler();
    // reverse the vector, since it returns in z, y, x
    euler = imu::Vector<3>(euler.x(), euler.y(), euler.z());
    return euler;
}

const char *VN_100::getCsvHeader() const
{                                                                                                    // incl VN- for Vector Nav
    return "VN-AX (m/s/s),VN-AY (m/s/s),VN-AZ (m/s/s),VN-ULRX (deg),VN-ULRY (deg),VN-ULRZ (deg),VN-QUATX,VN-QUATY,VN-QUATZ,VN-QUATW,VN-ANGVX (rad/s),VN-ANGVY (rad/s),VN-ANGVZ (rad/s),VN-MAGX (uT),VN-MAGY (uT),VN-MAGZ (uT),VN-P (Pa),VN-T (C),"; // trailing comma
}

const char *VN_100::getDataString() const
{
    sprintf(data, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", accelerationVec.x(), accelerationVec.y(), accelerationVec.z(), orientationEuler.x(), orientationEuler.y(), orientationEuler.z(), orientation.x(), orientation.y(), orientation.z(), orientation.w(), angularVelocity.x(), angularVelocity.y(), angularVelocity.z(), magnetometer.x(), magnetometer.y(), magnetometer.z(), pressure, temperature); // trailing comma"
    return data;
}

const char *VN_100::getStaticDataString() const
{
    sprintf(staticData, "None");
    return staticData; // TODO
}