#include "AirbrakeKF.h"

// Define the measurement size, control size, and state size
AirbrakeKF::AirbrakeKF() : LinearKalmanFilter(3, 3, 6) {}

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
    double *data = new double[18]{
        1.0, 0, 0, 0, 0, 0,
        0, 1.0, 0, 0, 0, 0,
        0, 0, 1.0, 0, 0, 0
    };
    return mmfs::Matrix(3, 6, data);
}

mmfs::Matrix AirbrakeKF::getR() {
    double *data = new double[9]{
        1.0, 0, 0,
        0, 1.0, 0,
        0, 0, 0.5
    };
    return mmfs::Matrix(3, 3, data);
}

mmfs::Matrix AirbrakeKF::getQ() {
    double *data = new double[36]{
        0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1
    };
    return mmfs::Matrix(6, 6, data);
}
