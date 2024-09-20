#include <Arduino.h>
#include <MMFS.h>

#include "airbrake_state.h"
#include "AirbrakeKF.h"

mmfs::BMP390 barometer;
mmfs::BNO055 ab_imu; 
mmfs::Sensor* airbrake_sensors[2] = {&barometer, &ab_imu};
AirbrakeKF kf;
mmfs::Logger logger;
AirbrakeState AIRBRAKE(airbrake_sensors, 2, &kf);

const int SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
const int SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

void setup() {

    logger.init();

    logger.recordLogData(mmfs::INFO_, "Entering Setup");


    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
}


double previous_time = millis(); // TODO should this live as functionality in MMFS?
double loop_time;

void loop() {
    loop_time = millis() - loop_time;
    previous_time = millis();
    AIRBRAKE.updateState();

    AIRBRAKE.update_cda_estimate();

    double tilt = ab_imu.getOrientationGlobal().x(); // TODO change this
    AIRBRAKE.calculateActuationAngle(AIRBRAKE.getPosition().z(), AIRBRAKE.getVelocity().z(), tilt, loop_time);
}