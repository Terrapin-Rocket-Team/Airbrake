#include <Arduino.h>
#include <MMFS.h>

#include "airbrake_state.h"

mmfs::Logger logger;

mmfs::BMP390 barometer;
mmfs::BNO055 ab_imu; 

mmfs::Sensor* airbrake_sensors[2] = {&barometer, &ab_imu};

mmfs::KalmanInterface kf(3, 3, 6);

AirbrakeState AIRBRAKE(airbrake_sensors, 2, &kf, &logger);


void setup() {
    logger.recordLogData(mmfs::INFO, "Entering Setup");


    logger.recordLogData(mmfs::INFO, "Leaving Setup");
}


double previous_time = millis(); // TODO should this live as functionality in MMFS?
double loop_time;

void loop() {
    loop_time = millis() - loop_time;
    previous_time = millis();
    AIRBRAKE.updateState();

    AIRBRAKE.update_cda_estimate();

    double tilt = ab_imu.getOrientationEuler().x(); // TODO change this
    AIRBRAKE.calculateActuationAngle(AIRBRAKE.getPosition().z(), AIRBRAKE.getVelocity().z(), tilt, loop_time);
}