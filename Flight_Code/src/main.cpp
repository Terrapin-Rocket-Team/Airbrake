#include <Arduino.h>

#include "airbrake_state.h"
#include "vn_100.h"
#include "AirbrakeKF.h"
#include "e5.h"
#include "BR.h"

// Testing
#define TEST_WITH_SERIAL

// Buzzer
const int BUZZER_PIN = 23;

// Encoder pins
const int enc_chan_a = 36;
const int enc_chan_b = 37;

// Sensors
E5 enc(enc_chan_a, enc_chan_b, "E5"); // Encoder
VN_100 vn(&SPI, 10); // Vector Nav
BR blueRaven;

#ifdef TEST_WITH_SERIAL
    char dataPath[2560];
    mmfs::MockBarometer mockDPS310(dataPath, "DPS310 - Pres (hPa)", "DPS310 - Temp (C)");
    mmfs::MockBarometer mockMS5611(dataPath, "MS5611 - Pres (hPa)", "MS5611 - Temp (C)");

    String accColNames[3] = { 
        String("BMI088andLIS3MDL - AccX"), 
        String("BMI088andLIS3MDL - AccY"), 
        String("BMI088andLIS3MDL - AccZ") 
    };
    String gyroColNames[3] = { 
        String("BMI088andLIS3MDL - GyroX"), 
        String("BMI088andLIS3MDL - GyroY"), 
        String("BMI088andLIS3MDL - GyroZ") 
    };
    String magColNames[3] = { 
        String("BMI088andLIS3MDL - MagX"), 
        String("BMI088andLIS3MDL - MagY"), 
        String("BMI088andLIS3MDL - MagZ") 
    };    
    mmfs::MockIMU mockBMI088andLIS3MDL(dataPath, accColNames, gyroColNames, magColNames);

    mmfs::MockGPS mockMAX_M10S(dataPath, "MAX-M10S - Lat", "MAX-M10S - Lon", "MAX-M10S - Alt (m)", "_", "MAX-M10S - Fix Quality");
    mmfs::Sensor* airbrake_sensors[7] = {&mockDPS310, &mockMS5611, &mockBMI088andLIS3MDL, &mockMAX_M10S, &enc, &vn, &blueRaven};
#else
    mmfs::DPS310 baro1; // Avionics Sensor Board 1.1
    mmfs::MS5611 baro2; // Avionics Sensor Board 1.1
    mmfs::BMI088andLIS3MDL airbrake_imu; // Avionics Sensor Board 1.1
    mmfs::MAX_M10S gps; // Avionics Sensor Board 1.1
    mmfs::Sensor* airbrake_sensors[7] = {&baro1, &baro2, &airbrake_imu, &gps, &enc, &vn, &blueRaven};
#endif

// Initialize Airbrake State
AirbrakeKF lkfmm;
AirbrakeState AIRBRAKE(airbrake_sensors, 7, &lkfmm);

// MMFS Stuff
mmfs::MMFSConfig config = mmfs::MMFSConfig()
                        .withState(&AIRBRAKE)
                        .withBuzzerPin(BUZZER_PIN)
                        .withUpdateRate(10);

mmfs::MMFSSystem sys(&config);

void setup() {
    // Initialize Serial and SPI Buses
    Serial.begin(115200);
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);
    SPI.begin();

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
    #ifdef TEST_WITH_SERIAL
    while(!Serial.available()){delay(100);}
    if (Serial.available()){
        Serial.readBytesUntil('\n', dataPath, sizeof(dataPath));
        Serial.println(dataPath);
    }
    #endif

    sys.init();

    #ifdef TEST_WITH_SERIAL
    #else
        baro1.setBiasCorrectionMode(true);
        baro2.setBiasCorrectionMode(true);
        gps.setBiasCorrectionMode(true);
    #endif
    

    // Limit Switch
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    if (enc.isInitialized()){
        AIRBRAKE.zeroMotor();
    }
    delay(5000);
    Serial.println("[][],0");
}

void loop() {
    #ifdef TEST_WITH_SERIAL
        if (Serial.available()){
            int i = Serial.readBytesUntil('\n', dataPath, sizeof(dataPath));
        } else {
            return;
        }
    #endif

    bool loop = sys.update();
    AIRBRAKE.updateMotor();
    AIRBRAKE.limitSwitchState = (digitalRead(LIMIT_SWITCH_PIN) == LOW);

    if(loop){
        #ifdef TEST_WITH_SERIAL
            Serial.printf("[][],%d\n", AIRBRAKE.actuationAngle);
            Serial.print("SD Baro1: ");
            Serial.print(mockDPS310.getPressure());
        #endif
        Serial.print("LKFMM Pos z: ");
        Serial.println(AIRBRAKE.getPosition().z());

        // Turn off bias correction during flight
        if (AIRBRAKE.stage == BOOST) {
            #ifdef TEST_WITH_SERIAL
            #else
                baro1.setBiasCorrectionMode(false);
                baro2.setBiasCorrectionMode(false);
                gps.setBiasCorrectionMode(false);
            #endif
        } else if (AIRBRAKE.stage == PRELAUNCH) {
            #ifdef TEST_WITH_SERIAL
            #else
                baro1.setBiasCorrectionMode(true);
                baro2.setBiasCorrectionMode(true);
                gps.setBiasCorrectionMode(true);
            #endif
        }
        if (AIRBRAKE.stage == COAST){
            AIRBRAKE.update_CdA_estimate();
        }
    }

    // // Test Deployment Code //
    // if (loop){
    //     if (millis() > 50000){
    //         Serial.print("Going to 0. Currently at: ");
    //         Serial.println(enc.getSteps());
    //         AIRBRAKE.goToDegree(0);  
    //         mmfs::getLogger().setRecordMode(mmfs::GROUND);
    //     }
    //     if (millis() > 30000){
    //         Serial.print("Going to 40. Currently at: ");
    //         Serial.println(enc.getSteps());
    //         mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
    //         AIRBRAKE.goToDegree(40);
    //     }
    // }
    

    // Flight Deployment Code //
    if (loop){
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

