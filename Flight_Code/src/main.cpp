#include <Arduino.h>

#include "airbrake_state.h"
#include "vn_100.h"
#include "AirbrakeKF.h"
#include "e5.h"
#include "BR.h"


// Buzzer
const int BUZZER_PIN = 23;

// Encoder pins
const int enc_chan_a = 36;
const int enc_chan_b = 37;

// Sensors
E5 enc(enc_chan_a, enc_chan_b, "E5"); // Encoder
VN_100 vn(&SPI, 10); // Vector Nav
mmfs::DPS310 baro1; // Avionics Sensor Board 1.1
mmfs::MS5611 baro2; // Avionics Sensor Board 1.1
mmfs::BMI088andLIS3MDL airbrake_imu; // Avionics Sensor Board 1.1
mmfs::MAX_M10S gps; // Avionics Sensor Board 1.1
BR blueRaven;
mmfs::Sensor* airbrake_sensors[7] = {&baro1, &baro2, &airbrake_imu, &gps, &enc, &vn, &blueRaven};

// Initialize Airbrake State
AirbrakeKF kf;
AirbrakeState AIRBRAKE(airbrake_sensors, 7, nullptr, BUZZER_PIN);

// MMFS Stuff
mmfs::MMFSConfig config = mmfs::MMFSConfig()
                        .withState(&AIRBRAKE)
                        .withBuzzerPin(BUZZER_PIN)
                        .withUpdateRate(25);

mmfs::MMFSSystem sys(&config);

void setup() {
    // Initialize Serial and SPI Buses
    Serial.begin(115200);
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);
    SPI.begin();

    if (CrashReport) Serial.println(CrashReport);

    // Immediately turn the motor off (needs the stop pin set to high)
    pinMode(brk_pin, OUTPUT);
    pinMode(stop_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(speed_pin, OUTPUT);
    digitalWrite(brk_pin, LOW);
    digitalWrite(stop_pin, HIGH);
    digitalWrite(dir_pin, LOW);
    analogWrite(speed_pin, 0);

    // MMFS Stuff
    sys.init();

    baro1.setBiasCorrectionMode(true);
    baro2.setBiasCorrectionMode(true);
    gps.setBiasCorrectionMode(true);
}

static unsigned long lastUpdateTime = 0;
int actuationAngle;
void loop() {

    sys.update();

    // // Turn off bias correction during flight
    if (AIRBRAKE.stage == BOOST) {
        baro1.setBiasCorrectionMode(false);
        baro2.setBiasCorrectionMode(false);
        gps.setBiasCorrectionMode(false);
    } else if (AIRBRAKE.stage == PRELAUNCH) {
        baro1.setBiasCorrectionMode(true);
        baro2.setBiasCorrectionMode(true);
        gps.setBiasCorrectionMode(true);
    }

    // // Test Deployment Code //
    // Serial.println(enc.getSteps());
    // if (millis() > 80000){
    //     logger.setRecordMode(mmfs::GROUND);
    //     AIRBRAKE.goToDegree(0);  
    // } else if (millis() > 40000){
    //     Serial.println("moving down");
    //     logger.setRecordMode(mmfs::FLIGHT);
    //     AIRBRAKE.goToDegree(40);
    // }

    // Flight Deployment Code //
    mmfs::Matrix dcm = AIRBRAKE.getOrientation().toMatrix();
    double tilt = acos(dcm.get(2,2));
    actuationAngle = AIRBRAKE.calculateActuationAngle(AIRBRAKE.getPosition().z(), AIRBRAKE.getVelocity().z(), tilt, UPDATE_RATE);
    if (AIRBRAKE.stage == DEPLOY){
        AIRBRAKE.goToDegree(actuationAngle);
    } else {
        AIRBRAKE.goToDegree(0);
    }

}

