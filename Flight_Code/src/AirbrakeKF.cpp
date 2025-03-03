#include "AirbrakeKF.h"

// Define the measurement size, control size, and state size
AirbrakeKF::AirbrakeKF() : LinearKalmanFilter(4, 3, 6) {}

mmfs::Matrix AirbrakeKF::getF(double dt) {
    double *data = new double[36]{
        1.0, 0, 0, dt, 0, 0,
        0, 1.0, 0, 0, dt, 0,
        0, 0, 1.0, 0, 0, dt,
        0, 0, 0, 1.0, 0, 0,
        0, 0, 0, 0, 1.0, 0,
        0, 0, 0, 0, 0, 1.0
    };
    return mmfs::Matrix(6, 6, data);
}

mmfs::Matrix AirbrakeKF::getG(double dt) {
    double *data = new double[18]{
        0.5 * dt * dt, 0, 0,
        0, 0.5 * dt * dt, 0,
        0, 0, 0.5 * dt * dt,
        dt, 0, 0,
        0, dt, 0,
        0, 0, dt
    };
    return mmfs::Matrix(6, 3, data);
}

mmfs::Matrix AirbrakeKF::getH() {
    double *data = new double[24]{
        1.0, 0, 0, 0, 0, 0,
        0, 1.0, 0, 0, 0, 0,
        0, 0, 1.0, 0, 0, 0,
        0, 0, 1.0, 0, 0, 0,
    };
    return mmfs::Matrix(4, 6, data);
}

mmfs::Matrix AirbrakeKF::getR() {
    double *data = new double[16]{
        gpsMAX_std, 0, 0, 0,
        0, gpsMAX_std, 0, 0,
        0, 0, dps310_std, 0,
        0, 0, 0, ms5611_std
    };
    return mmfs::Matrix(4, 4, data);
}

mmfs::Matrix AirbrakeKF::getQ(double dt) {
    double *data = new double[36]{
        std::pow(dt, 4)/4, 0, 0, std::pow(dt, 3)/2, 0, 0,
        0, std::pow(dt, 4)/4, 0, 0, std::pow(dt, 3)/2, 0,
        0, 0, std::pow(dt, 4)/4, 0, 0, std::pow(dt, 3)/2,
        std::pow(dt, 3)/2, 0, 0, std::pow(dt, 2), 0, 0,
        0, std::pow(dt, 3)/2, 0, 0, std::pow(dt, 2), 0,
        0, 0, std::pow(dt, 3)/2, 0, 0, std::pow(dt, 2)
    };
    return mmfs::Matrix(6, 6, data)*processNoise*processNoise;
}

