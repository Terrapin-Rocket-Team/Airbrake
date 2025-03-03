#ifndef AIRBRAKE_KF_H
#define AIRBRAKE_KF_H

#include <Filters/LinearKalmanFilter.h>



class AirbrakeKF : public mmfs::LinearKalmanFilter {
public:
    AirbrakeKF();
    ~AirbrakeKF() = default;

    // Measurement noise
    double gpsMAX_std = 1; // [m]
    double dps310_std = .2; // [m]
    double ms5611_std = .5; // [m]
    
    double processNoise = .5; // [m]

    // Override getter methods to provide subteam-specific matrix implementations
    void initialize() override {};
    mmfs::Matrix getF(double dt) override;
    mmfs::Matrix getG(double dt) override;
    mmfs::Matrix getH() override;
    mmfs::Matrix getR() override;
    mmfs::Matrix getQ(double dt) override;
};


#endif // AIRBRAKE_KF_H
