#include "vn_100.h"
#include <RecordData/DataReporter.h>

bool VN_100::begin(bool useBiasCorrection)
{
    biasCorrectionMode = useBiasCorrection;
    return init();
}

bool VN_100::init()
{
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
    packData();
}

void VN_100::read()
{   
    if (vn.Read()) {
        accelerationVec = imu::Vector<3>(vn.accel_x_mps2(), vn.accel_y_mps2(), vn.accel_z_mps2());
        angularVelocity = imu::Vector<3>(vn.gyro_x_radps(), vn.gyro_y_radps(), vn.gyro_z_radps());
        orientationEuler = imu::Vector<3>(vn.yaw_rad() * 180 / 3.14159, vn.pitch_rad() * 180 / 3.14159, vn.roll_rad() * 180 / 3.14159);
        magnetometer = imu::Vector<3>(vn.mag_x_ut(), vn.mag_y_ut(), vn.mag_z_ut());
        deltaVelocity = imu::Vector<3>(vn.delta_velocity_x(), vn.delta_velocity_y(), vn.delta_velocity_z());
        deltaTheta = imu::Vector<3>(vn.delta_theta_x(), vn.delta_theta_y(), vn.delta_theta_z());
        deltaTime = vn.delta_time();
        
        // TODO quaterion orientation
        pressure = vn.pres_pa();
        temperature = vn.die_temp_c();
    }
}

void VN_100::calibrate()
{
    // TODO
}

imu::Quaternion VN_100::getOrientation() const
{
    return orientation;
}

imu::Vector<3> VN_100::getAcceleration() const
{
    return accelerationVec;
}

imu::Vector<3> VN_100::getOrientationEuler() const
{
    return orientationEuler;
}

imu::Vector<3> VN_100::getMagnetometer() const
{
    return magnetometer;
}

imu::Vector<3> VN_100::getDeltaVelocity() const
{
    return deltaVelocity;
}

imu::Vector<3> VN_100::getDeltaTheta() const 
{
    return deltaTheta;
}

double VN_100::getDeltaTime() const
{
    return deltaTime;
}

imu::Vector<3> convertToEuler(const imu::Quaternion &orientation)
{
    imu::Vector<3> euler = orientation.toEuler();
    // reverse the vector, since it returns in z, y, x
    euler = imu::Vector<3>(euler.x(), euler.y(), euler.z());
    return euler;
}

const int VN_100::getNumPackedDataPoints() const
{
    return 21;
}

const mmfs::PackedType *VN_100::getPackedOrder() const
{
    static const mmfs::PackedType result[] = {mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT,
            mmfs::FLOAT,
            mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT,
            mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT,
            mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT, mmfs::FLOAT};

    return result;
}

const char **VN_100::getPackedDataLabels() const
{
    static const char *labels[] = {"VN-AX (m/s/s)", "VN-AY (m/s/s)", "VN-AZ (m/s/s)", "VN-ULRX (deg)", "VN-ULRY (deg)",
                            "VN-ULRZ (deg)",
                            "VN-ANGVX (rad/s)", "VN-ANGVY (rad/s)", "VN-ANGVZ (rad/s)", "VN-MAGX (uT)", "VN-MAGY (uT)",
                            "VN-MAGZ (uT)", "VN-P (Pa)", "VN-T (C)", "VN-DT (s)", "VN-DV-X (m/s/s)",
                            "VN-DV-Y (m/s/s)", "VN-DV-Z (m/s/s)", "VN-DTH-X (deg/s)", "VN-DTH-Y (deg/s)", "VN-DTH-Z (deg/s)"};
    
    return labels;
}

void VN_100::packData()
{
    struct PackedData data;
    data.ax = accelerationVec.x();
    data.ay = accelerationVec.y();
    data.az = accelerationVec.z();
    data.ulrx = orientationEuler.x();
    data.ulry = orientationEuler.y();
    data.ulrz = orientationEuler.z();
    data.angvx = angularVelocity.x();
    data.angvy = angularVelocity.y();
    data.angvz = angularVelocity.z();
    data.magx = magnetometer.x();
    data.magy = magnetometer.y();
    data.magz = magnetometer.z();
    data.p = pressure;
    data.t = temperature;
    data.dt = deltaTime;
    data.dv_x = deltaVelocity.x();
    data.dv_y = deltaVelocity.y();
    data.dv_z = deltaVelocity.z();
    data.dth_x = deltaTheta.x();
    data.dth_y = deltaTheta.y();
    data.dth_z = deltaTheta.z();
    memcpy(packedData, &data, sizeof(PackedData));
}
