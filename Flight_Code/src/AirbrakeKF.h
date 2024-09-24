#ifndef AIRBRAKE_KF_H
#define AIRBRAKE_KF_H

#include <Filters/LinearKalmanFilter.h>



class AirbrakeKF : public mmfs::LinearKalmanFilter {
public:
    AirbrakeKF();
    ~AirbrakeKF() = default;

    // Override getter methods to provide subteam-specific matrix implementations
    void initialize() override {};
    mmfs::Matrix getF(double dt) override;
    mmfs::Matrix getG(double dt) override;
    mmfs::Matrix getH() override;
    mmfs::Matrix getR() override;
    mmfs::Matrix getQ() override;
};


#endif // AIRBRAKE_KF_H
