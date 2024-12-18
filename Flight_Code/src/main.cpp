#include <Arduino.h>

#include "airbrake_state.h"
#include "vn_100.h"
#include "AirbrakeKF.h"
#include "e5.h"

const int BUZZER_PIN = 1; //TODO changes this
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

// Encoder pins
const int enc_chan_a = 36;
const int enc_chan_b = 37;

// Motor driver pins
const int brk_pin = 3; 
const int dir_pin = 5; 

E5 enc(enc_chan_a, enc_chan_b, "E5");
VN_100 vn(&SPI, 10);

mmfs::DPS310 baro1;
mmfs::MS5611 baro2;
mmfs::BMI088andLIS3MDL airbrake_imu;
mmfs::MAX_M10S gps;

mmfs::Sensor* airbrake_sensors[6] = {&baro1, &baro2, &airbrake_imu, &gps, &enc, &vn};
AirbrakeKF kf;
mmfs::Logger logger;
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
AirbrakeState AIRBRAKE(airbrake_sensors, 6, &kf);

const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

void setup() {
    Serial.begin(115200);
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);
    SPI.begin();

    pinMode(brk_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    digitalWrite(brk_pin, LOW);
    digitalWrite(dir_pin, HIGH);

    SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
    SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;

    psram = new mmfs::PSRAM();

    logger.init(&AIRBRAKE);

    AIRBRAKE.init();


    logger.recordLogData(mmfs::INFO_, "Entering Setup");


    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
}


void loop() {



    AIRBRAKE.updateState();

    //AIRBRAKE.update_cda_estimate();

    Serial.println(enc.getSteps());
    //double tilt = ab_imu.getOrientationGlobal().x(); // TODO change this
    //AIRBRAKE.calculateActuationAngle(AIRBRAKE.getPosition().z(), AIRBRAKE.getVelocity().z(), tilt, loop_time);
}