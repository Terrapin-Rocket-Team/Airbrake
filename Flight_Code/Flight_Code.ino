#include <Arduino.h>
#include <MMFS.h>

#include "airbrake_state.h"

mmfs::BMP390 barometer;
mmfs::BNO055 imu; 

mmfs::Sensor* airbrake_sensors[2] = {&barometer, &imu};

mmfs::KalmanInterface kf(2, 2, 2);

AirbrakeState AIRBRAKE(airbrake_sensors, 2, &kf);


void setup() {


}

void loop() {


    AIRBRAKE.calculateActuationAngle()


}