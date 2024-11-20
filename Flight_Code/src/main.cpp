#include <Arduino.h>
#include <MMFS.h>

#include "airbrake_state.h"
#include "vn_100.h"
#include "AirbrakeKF.h"
#include "e5.h"

const int enc_chan_a = 36;
const int enc_chan_b = 37;

E5 enc(enc_chan_a, enc_chan_b);
mmfs::BMP390 barometer;
mmfs::BNO055 ab_imu; 
VN_100 vn(&SPI, 10);

mmfs::Sensor* airbrake_sensors[3] = {&barometer, &ab_imu, &vn};
AirbrakeKF kf;
mmfs::Logger logger;
AirbrakeState AIRBRAKE(airbrake_sensors, 3, &kf);
const int SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
const int SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

void setup() {
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);
    SPI.begin();


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