#include <Arduino.h>

#include "airbrake_state.h"
#include "AirbrakeKF.h"
#include "e5.h"
#include "BR.h"
#include "Events/Event.h"
#include "RetrieveData/SerialHandler.h"
#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include <Radio/ESP32BluetoothRadio.h>
#include "MockBR.h"

// TODO: Long List
// 2(b). Add flap deployment pressure spike for input barometer data (see Ezra's paper)

// Testing
// #define TEST_WITH_SERIAL

// Bluetooth Module
APRSConfig aprsConfig = {"KC3UTM", "ALL", "WIDE1-1", PositionWithoutTimestampWithoutAPRS, '\\', 'M'};
uint8_t encoding[] = {7, 4, 5, 7, 8};
APRSTelem aprs(aprsConfig);
Message msg;
mmfs::ESP32BluetoothRadio btRad(Serial5, "AVIONICS", true);

// Buzzer
const int BUZZER_PIN = 23;

// Encoder pins
const int enc_chan_a = 36;
const int enc_chan_b = 37;

// Sensors
E5 enc(enc_chan_a, enc_chan_b, "E5"); // Encoder

#ifdef TEST_WITH_SERIAL

char dataPath[2560];
bool firstLineReceived = false;

mmfs::MockBarometer mockDPS310(dataPath, "DPS310 - Pres (hPa)", "DPS310 - Temp (C)");

std::string accColNames[3] = {
    std::string("BMI088andLIS3MDL - AccX"),
    std::string("BMI088andLIS3MDL - AccY"),
    std::string("BMI088andLIS3MDL - AccZ")};
std::string gyroColNames[3] = {
    std::string("BMI088andLIS3MDL - GyroX"),
    std::string("BMI088andLIS3MDL - GyroY"),
    std::string("BMI088andLIS3MDL - GyroZ")};
std::string magColNames[3] = {
    std::string("BMI088andLIS3MDL - MagX"),
    std::string("BMI088andLIS3MDL - MagY"),
    std::string("BMI088andLIS3MDL - MagZ")};
mmfs::MockIMU mockBMI088andLIS3MDL(dataPath, accColNames, gyroColNames, magColNames);

mmfs::MockGPS mockMAX_M10S(dataPath, "MAX-M10S - Lat", "MAX-M10S - Lon", "MAX-M10S - Alt (m)", "_", "MAX-M10S - Fix Quality");

std::string brAccColNames[3] = {
    std::string("BR - ACCX (m/s^2)"),
    std::string("BR - ACCY (m/s^2)"),
    std::string("BR - ACCZ (m/s^2)")};
std::string brGyroColNames[3] = {
    std::string("BR - GYROX (rad/s)"),
    std::string("BR - GYROY (rad/s)"),
    std::string("BR - GYROZ (rad/s)")};
mmfs::MockBR mockBlueRaven(dataPath, "BR - ALT (m)", "BR - PRES (Pa)", "BR - TEMP (C)", "BR - TILT (deg)", "BR - ROLL (deg)", "BR - VEL (m/s)", brAccColNames, brGyroColNames);

mmfs::Sensor *airbrake_sensors[5] = {&mockDPS310, &mockBMI088andLIS3MDL, &mockMAX_M10S, &enc, &mockBlueRaven};
#else
mmfs::DPS368 baro1; // Avionics Sensor Board 1.2
mmfs::BMI088andLIS3MDL airbrake_imu; // Avionics Sensor Board 1.2
mmfs::MAX_M10S gps;                  // Avionics Sensor Board 1.2
BR blueRaven;
mmfs::Sensor *airbrake_sensors[5] = {&baro1, &airbrake_imu, &gps, &enc, &blueRaven};
#endif

// // Initialize Airbrake State
AirbrakeKF lkfmm;
AirbrakeState AIRBRAKE(airbrake_sensors, sizeof(airbrake_sensors) / 4, &lkfmm);

// // MMFS Stuff
mmfs::MMFSConfig config = mmfs::MMFSConfig()
                              .withState(&AIRBRAKE)
                              .withBuzzerPin(BUZZER_PIN)
                              .withUpdateRate(10);

mmfs::MMFSSystem sys(&config);

void setup()
{
    // Initialize Serial and SPI Buses
    Serial.begin(115200);

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
        char startBuffer[100]; // Buffer for incoming data
        String receivedCommand = "";
        // Wait until "Sim Start," is received
        while (true)
        {
            if (Serial.available())
            {
                // Read incoming data into buffer
                Serial.readBytesUntil('\n', startBuffer, sizeof(startBuffer));

                // Convert char array to String
                receivedCommand = String(startBuffer);
                receivedCommand.trim(); // Remove any extra whitespace/newlines

                // Check if received command matches "Sim Start,"
                if (receivedCommand == "telem/Sim Start")
                {
                    Serial.println("Sim Start Received");
                    break; // Exit loop and proceed
                }
            }
            delay(100); // Prevent excessive CPU usage
        }
        if (Serial.available())
        {
            Serial.readBytesUntil('\n', dataPath, sizeof(dataPath));
            Serial.println(dataPath);
            mmfs::getLogger().recordLogData(mmfs::INFO_, "This is a simulation run.");
        }
    #endif

    #ifdef TEST_WITH_SERIAL
        mockDPS310.setBiasCorrectionMode(true);
        mockMAX_M10S.setBiasCorrectionMode(true);
    #else
        baro1.setBiasCorrectionMode(true);
        gps.setBiasCorrectionMode(true);
    #endif

    sys.init();

    // Limit Switch
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    if (enc.isInitialized())
    {   
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Zeroing Motor.");
        AIRBRAKE.zeroMotor();
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Motor Zeroed.");
    }
    delay(1000);
    #ifdef TEST_WITH_SERIAL
        Serial.println("[][],0");
    #endif

    if (btRad.begin())
    {
        bb.onoff(mmfs::BUZZER, 500); // 1 x 0.5 sec beep for sucessful initialization
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Initialized Bluetooth");
    }
    else
    {
        bb.onoff(mmfs::BUZZER, 1000, 3); // 3 x 2 sec beep for uncessful initialization
        mmfs::getLogger().recordLogData(mmfs::ERROR_, "Initialized Bluetooth Failed");
    }
}

int btLast = millis();

void loop()
{
    #ifdef TEST_WITH_SERIAL
        if (Serial.available()){
            Serial.readBytesUntil('\n', dataPath, sizeof(dataPath));
            Serial.println(dataPath);
        }
    #endif

    bool doLoop = sys.update();
    AIRBRAKE.actualAngle = AIRBRAKE.stepToDegree(enc.getSteps());
    AIRBRAKE.updateMotor();

    if (doLoop)
    {
        // Turn off bias correction during flight
        if (AIRBRAKE.stage == BOOST)
        {
            #ifdef TEST_WITH_SERIAL
                mockDPS310.setBiasCorrectionMode(false);
                mockMAX_M10S.setBiasCorrectionMode(false);
            #else
                baro1.setBiasCorrectionMode(false);
                gps.setBiasCorrectionMode(false);
            #endif
        }
        else if (AIRBRAKE.stage == PRELAUNCH)
        {
            #ifdef TEST_WITH_SERIAL
                mockDPS310.setBiasCorrectionMode(true);
                mockMAX_M10S.setBiasCorrectionMode(true);
            #else
                baro1.setBiasCorrectionMode(true);
                gps.setBiasCorrectionMode(true);
            #endif
        }
        if (AIRBRAKE.stage == COAST)
        {
            AIRBRAKE.update_CdA_estimate();
        }
    }

    // Test Deployment Code //

    // if (doLoop){
    //     // if (millis() > 30000){
    //     //     AIRBRAKE.goToDegree(0);
    //     //     mmfs::getLogger().setRecordMode(mmfs::GROUND);
    //     // }
    //     if (millis() > 20000){
    //         // mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
    //         AIRBRAKE.goToDegree(65);
    //         Serial.println(enc.getSteps());
    //     }
    // }

    // Flight Deployment Code //

    if (doLoop)
    {
    //    Serial.println(AIRBRAKE.getPosition().z());
       mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(AIRBRAKE.getSensor("Barometer"_i));
       mmfs::Matrix dcm = AIRBRAKE.getOrientation().conjugate().toMatrix();
       double tilt = acos(dcm.get(2, 2)); // [rad]
       tilt = M_PI / 2 - tilt;            // 90 deg off for some reason TODO figure out
       AIRBRAKE.tilt = tilt * 180 / M_PI; // [deg]
       Serial.println("Tilt" + String(AIRBRAKE.tilt));
    //    Serial.printf("Tilt: %f\n", AIRBRAKE.tilt);
    //    Serial.printf("Sensor Acc Glob Z: %f\n", AIRBRAKE.getAcceleration().z());
       double velocity = AIRBRAKE.getVelocity().magnitude();
       double altitude = AIRBRAKE.getPosition().z();
       if (AIRBRAKE.stage == DEPLOY)
       {

           int actuationAngle = AIRBRAKE.calculateActuationAngle(altitude, velocity, tilt);
           AIRBRAKE.estimated_apogee = AIRBRAKE.predict_apogee(.05, tilt, velocity, altitude, AIRBRAKE.actualAngle);
           AIRBRAKE.goToDegree(actuationAngle);
       }
       else
       {
           AIRBRAKE.goToDegree(0);
       }

    //    If not going to hit expected apogee still try to take some altitude off
    //    if (AIRBRAKE.stage == COAST){
    //        double estimated_apogee = AIRBRAKE.predict_apogee(.5, tilt, velocity, altitude);
    //        if (estimated_apogee < (AIRBRAKE.predicted_target_apogee + 500)) {
    //            AIRBRAKE.target_apogee = estimated_apogee - 500;
    //        }
    //    }

        #ifdef TEST_WITH_SERIAL
            // Used for only software testing
            // double flapSpeed = 25; // speed at which the flaps open [deg/s]
            // double desiredDegree = AIRBRAKE.stepToDegree(AIRBRAKE.desiredStep);
            // int sign = 0;
            // if (desiredDegree > AIRBRAKE.actualAngle) sign = 1;
            // else if (desiredDegree < AIRBRAKE.actualAngle) sign = -1;

            // AIRBRAKE.actualAngle += (sign * flapSpeed * (UPDATE_INTERVAL / 1000.0));

            // // Clamp to avoid overshoot
            // if ((sign > 0 && AIRBRAKE.actualAngle > desiredDegree) ||
            // (sign < 0 && AIRBRAKE.actualAngle < desiredDegree)) {
            // AIRBRAKE.actualAngle = desiredDegree;
            // }

            // Serial.printf("[][],%d\n", (int)AIRBRAKE.actualAngle); 

            // // Used for encoder in the loop testing
            Serial.printf("[][],%d\n", AIRBRAKE.stepToDegree(enc.getSteps())); 
        #endif
    }   

    // Bluetooth Stuff //
    if (doLoop)
    {
        #ifndef TEST_WITH_SERIAL
        /// printf("%f\n", baro1.getAGLAltFt());
        aprs.alt = baro1.getAGLAltFt();
        // printf("%f\n", gps.getHeading());
        aprs.hdg = 0.0;
        // printf("%f\n", gps.getPos().x());
        aprs.lat = 0.0;
        // printf("%f\n", gps.getPos().y());
        aprs.lng = 0.0;
        // printf("%f\n", computer.getVelocity().z());
        aprs.spd = AIRBRAKE.getVelocity().z();
        // printf("%f\n", bno.getAngularVelocity().x());
        aprs.orient[0] = airbrake_imu.getAngularVelocity().x();
        // printf("%f\n", bno.getAngularVelocity().y());
        aprs.orient[1] = airbrake_imu.getAngularVelocity().y();
        // printf("%f\n", bno.getAngularVelocity().z());
        aprs.orient[2] = airbrake_imu.getAngularVelocity().z();
        aprs.stateFlags.setEncoding(encoding, 3);
        uint8_t arr[] = {(uint8_t)(int)baro1.getTemp(), (uint8_t)AIRBRAKE.getStage(), (uint8_t)AIRBRAKE.actualAngle, (uint8_t)((int)AIRBRAKE.estimated_apogee << 8), (uint8_t)((int)AIRBRAKE.estimated_apogee & 0xff)};
        aprs.stateFlags.pack(arr);
        // Serial.printf("%f %ld\n", baro1.getTemp(), aprs.stateFlags.get());
        btRad.send(aprs);
        #endif
    }
}

