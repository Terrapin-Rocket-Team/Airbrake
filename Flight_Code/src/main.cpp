#include <Arduino.h>

#include "airbrake_state.h"
#include "vn_100.h"
#include "AirbrakeKF.h"
#include "e5.h"
#include "BR.h"

// Testing
//#define TEST_WITH_SD_DATA

// Buzzer
const int BUZZER_PIN = 23;

// Encoder pins
const int enc_chan_a = 36;
const int enc_chan_b = 37;

// Sensors
E5 enc(enc_chan_a, enc_chan_b, "E5"); // Encoder
VN_100 vn(&SPI, 10); // Vector Nav
mmfs::BMI088andLIS3MDL airbrake_imu; // Avionics Sensor Board 1.1
mmfs::MAX_M10S gps; // Avionics Sensor Board 1.1
BR blueRaven;

#ifdef TEST_WITH_SD_DATA
    const char* dataPath = "Jan_Airbrake_FlightData.csv";
    mmfs::MockBarometer mockDPS310(dataPath, "DPS310-Pres (hPa)", "DPS310-Temp (C)");
    mmfs::MockBarometer mockMS5611(dataPath, "MS5611-Pres (hPa)", "MS5611-Temp (C)");
    mmfs::Sensor* airbrake_sensors[7] = {&mockDPS310, &mockMS5611, &airbrake_imu, &gps, &enc, &vn, &blueRaven};
#else
    mmfs::DPS310 baro1; // Avionics Sensor Board 1.1
    mmfs::MS5611 baro2; // Avionics Sensor Board 1.1
    mmfs::Sensor* airbrake_sensors[7] = {&baro1, &baro2, &airbrake_imu, &gps, &enc, &vn, &blueRaven};
#endif

// Initialize Airbrake State
AirbrakeKF lkfmm;
AirbrakeState AIRBRAKE(airbrake_sensors, 7, nullptr);

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

    #ifdef TEST_WITH_SD_DATA
    #else
        baro1.setBiasCorrectionMode(true);
        baro2.setBiasCorrectionMode(true);
        gps.setBiasCorrectionMode(true);
    #endif
    

    // Limit Switch
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    delay(5000);
    if (enc.isInitialized()){
        AIRBRAKE.zeroMotor();
    }

    bb.onoff(BUZZER_PIN, 1000);
}

void FreeMem()
{
    void *heapTop = malloc(2000);
    int stack = 0;
    long dif = ((long)heapTop) - ((long) &stack);
    Serial.print(dif);
    Serial.print("\n");
    free(heapTop);
}
void loop() {
    //FreeMem();
    bool loop = sys.update();
    AIRBRAKE.updateMotor();
    AIRBRAKE.limitSwitchState = (digitalRead(LIMIT_SWITCH_PIN) == LOW);

    // if (loop) {
    //     Serial.print("LKF Pos z: ");
    //     Serial.println(AIRBRAKE.getPosition().z());
    //     Serial.println(enc.getSteps());
    // }
    // Turn off bias correction during flight
    if (AIRBRAKE.stage == BOOST) {
        #ifdef TEST_WITH_SD_DATA
        #else
            baro1.setBiasCorrectionMode(false);
            baro2.setBiasCorrectionMode(false);
            gps.setBiasCorrectionMode(false);
        #endif
    } else if (AIRBRAKE.stage == PRELAUNCH) {
        #ifdef TEST_WITH_SD_DATA
        #else
            baro1.setBiasCorrectionMode(true);
            baro2.setBiasCorrectionMode(true);
            gps.setBiasCorrectionMode(true);
        #endif
    }

    if (AIRBRAKE.stage == COAST){
        AIRBRAKE.update_CdA_estimate();
    }

    // // Test Deployment Code //
    // if (millis() > 80000){
    //     Serial.print("Going to 0. Currently at: ");
    //     Serial.println(enc.getSteps());
    //     //AIRBRAKE.goToDegree(0);  
    //     mmfs::getLogger().setRecordMode(mmfs::GROUND);
    if (millis() > 30000){
        Serial.print("Going to 40. Currently at: ");
        Serial.println(enc.getSteps());
        mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
        //AIRBRAKE.goToDegree(70);
        AIRBRAKE.goToStep(-750000);
    }

    // Flight Deployment Code //

    if (loop) {
        mmfs::Matrix dcm = AIRBRAKE.getOrientation().toMatrix();
        double tilt = acos(dcm.get(2,2));
        double velocity = AIRBRAKE.getVelocity().magnitude();
        int actuationAngle = AIRBRAKE.calculateActuationAngle(AIRBRAKE.getPosition().z(), velocity, tilt, UPDATE_INTERVAL/1000);
        if (AIRBRAKE.stage == DEPLOY){
            AIRBRAKE.goToDegree(actuationAngle);
        } else {
            AIRBRAKE.goToDegree(0);
        }
    }
    

}

