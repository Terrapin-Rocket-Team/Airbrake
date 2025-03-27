#include <Arduino.h>

#include "airbrake_state.h"
#include "vn_100.h"
#include "AirbrakeKF.h"
#include "e5.h"
#include "BR.h"
#include "Events/Event.h"
#include "RetrieveData/SerialHandler.h"
#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include <Radio/ESP32BluetoothRadio.h>


// TODO: Long List
// 1. Make the kalman filter be able to handle no GPS. We won't get any
// 2. Fix the CdA calculations. Use the vector nav z direction
// 3. Add the blue raven as a working recording sensor
// 4. Add the vector nav and blue raven in the hardware in the loop testing
// 5. Figure out why the altitude estimation is undershooting it
// 6. Add tilt to the HITL and test

// Testing
#define TEST_WITH_SERIAL

// Bluetooth Module
APRSConfig aprsConfig = {"KC3UTM", "ALL", "WIDE1-1", PositionWithoutTimestampWithoutAPRS, '\\', 'M'};
uint8_t encoding[] = {7, 4, 4};
ESP32BluetoothRadio btTransmitter(Serial1, "AIRBRAKE", true);
APRSTelem bt_aprs(aprsConfig);
Message bt_msg;

// Buzzer
const int BUZZER_PIN = 23;

// Encoder pins
const int enc_chan_a = 36;
const int enc_chan_b = 37;

// Sensors
E5 enc(enc_chan_a, enc_chan_b, "E5"); // Encoder
VN_100 vn(&SPI, 10);                  // Vector Nav
BR blueRaven;

#ifdef TEST_WITH_SERIAL

char dataPath[2560];
bool firstLineReceived = false;
void onSerialEvent(const mmfs::Event &e)
{
    using namespace mmfs;
    if (e.ID == "SERIAL_LINE"_i)
    {
        if (strncmp("telem/", getSerialHandler().getLastLine(), 6) == 0)
        {
            strcpy(dataPath, getSerialHandler().getLastLine()+6);
            firstLineReceived = true;
        }
    }
}
mmfs::MockBarometer mockDPS310(dataPath, "DPS310 - Pres (hPa)", "DPS310 - Temp (C)");
// mmfs::MockBarometer mockMS5611(dataPath, "MS5611 - Pres (hPa)", "MS5611 - Temp (C)");

String accColNames[3] = {
    String("BMI088andLIS3MDL - AccX"),
    String("BMI088andLIS3MDL - AccY"),
    String("BMI088andLIS3MDL - AccZ")};
String gyroColNames[3] = {
    String("BMI088andLIS3MDL - GyroX"),
    String("BMI088andLIS3MDL - GyroY"),
    String("BMI088andLIS3MDL - GyroZ")};
String magColNames[3] = {
    String("BMI088andLIS3MDL - MagX"),
    String("BMI088andLIS3MDL - MagY"),
    String("BMI088andLIS3MDL - MagZ")};
mmfs::MockIMU mockBMI088andLIS3MDL(dataPath, accColNames, gyroColNames, magColNames);

mmfs::MockGPS mockMAX_M10S(dataPath, "MAX-M10S - Lat", "MAX-M10S - Lon", "MAX-M10S - Alt (m)", "_", "MAX-M10S - Fix Quality");
mmfs::Sensor *airbrake_sensors[6] = {&mockDPS310, &mockBMI088andLIS3MDL, &mockMAX_M10S, &enc, &vn, &blueRaven};
#else
mmfs::DPS310 baro1; // Avionics Sensor Board 1.1
// mmfs::MS5611 baro2; // Avionics Sensor Board 1.1
mmfs::BMI088andLIS3MDL airbrake_imu; // Avionics Sensor Board 1.1
mmfs::MAX_M10S gps;                  // Avionics Sensor Board 1.1
mmfs::Sensor *airbrake_sensors[6] = {&baro1, &airbrake_imu, &gps, &enc, &vn, &blueRaven};
#endif

// Initialize Airbrake State
AirbrakeKF lkfmm;
AirbrakeState AIRBRAKE(airbrake_sensors, sizeof(airbrake_sensors) / 4, &lkfmm);

// MMFS Stuff
mmfs::MMFSConfig config = mmfs::MMFSConfig()
                              .withState(&AIRBRAKE)
                              .withBuzzerPin(BUZZER_PIN)
                              .withUpdateRate(10);

mmfs::MMFSSystem sys(&config);

void setup()
{
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
        char title[2560];
        int i = Serial.readBytesUntil('\n', title, sizeof(title));
        title[i] = '\0';
        strcpy(dataPath, title+6);
        Serial.println(dataPath);
        mmfs::getLogger().recordLogData(mmfs::INFO_, "This is a simulation run.");
    }
#endif

#ifdef TEST_WITH_SERIAL
    mockDPS310.setBiasCorrectionMode(true);
    // mockMS5611.setBiasCorrectionMode(true);
    mockMAX_M10S.setBiasCorrectionMode(true);
#else
    baro1.setBiasCorrectionMode(true);
    // baro2.setBiasCorrectionMode(true);
    gps.setBiasCorrectionMode(true);
#endif

    sys.init();

    // Limit Switch
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    if (enc.isInitialized())
    {
        AIRBRAKE.zeroMotor();
    }
    delay(1000);
    Serial.println("[][],0");

    if (btTransmitter.begin()) {
        getLogger().recordLogData(INFO_, "Initialized Bluetooth");
    } else {
        getLogger().recordLogData(ERROR_, "Initialized Bluetooth Failed");
    }
}

int btLast = millis();

void loop()
{
#ifdef TEST_WITH_SERIAL
    if (!firstLineReceived)
    {
        return;
    }
#endif

    bool loop = sys.update();
    AIRBRAKE.updateMotor();
    AIRBRAKE.limitSwitchState = (digitalRead(LIMIT_SWITCH_PIN) == LOW);

    if (loop)
    {
        // Turn off bias correction during flight
        if (AIRBRAKE.stage == BOOST)
        {
#ifdef TEST_WITH_SERIAL
            mockDPS310.setBiasCorrectionMode(false);
            // mockMS5611.setBiasCorrectionMode(false);
            mockMAX_M10S.setBiasCorrectionMode(false);
#else
            baro1.setBiasCorrectionMode(false);
            baro2.setBiasCorrectionMode(false);
            gps.setBiasCorrectionMode(false);
#endif
        }
        else if (AIRBRAKE.stage == PRELAUNCH)
        {
#ifdef TEST_WITH_SERIAL
            mockDPS310.setBiasCorrectionMode(true);
            // mockMS5611.setBiasCorrectionMode(true);
            mockMAX_M10S.setBiasCorrectionMode(true);
#else
            baro1.setBiasCorrectionMode(true);
            baro2.setBiasCorrectionMode(true);
            gps.setBiasCorrectionMode(true);
#endif
        }
        if (AIRBRAKE.stage == COAST)
        {
            // AIRBRAKE.update_CdA_estimate();
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

    if (loop)
    {
        mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(AIRBRAKE.getSensor(mmfs::BAROMETER_));
        AIRBRAKE.machNumber = AIRBRAKE.getVelocity().magnitude() / sqrt(1.4 * 286 * (baro->getTemp() + 273.15)); // M = V/sqrt(gamma*R*T)
        mmfs::Matrix dcm = AIRBRAKE.getOrientation().toMatrix();
        double tilt = acos(dcm.get(2, 2));            // [rad]
        AIRBRAKE.tilt = 90 - (tilt * (180.0 / M_PI)); // [deg]
        Serial.printf("Tilt: %f\n", AIRBRAKE.tilt);
        if (AIRBRAKE.stage == DEPLOY)
        {
            double velocity = AIRBRAKE.getVelocity().magnitude();
            int actuationAngle = AIRBRAKE.calculateActuationAngle(AIRBRAKE.getPosition().z(), velocity, M_PI / 2 - tilt);
            AIRBRAKE.goToDegree(actuationAngle);

        }
        else
        {
            AIRBRAKE.goToDegree(0);
        }

        if (millis() - btLast > 1000)
        {
            btLast = millis();
            bt_aprs.alt = AIRBRAKE.getPosition().z() * 3.28084; // Convert to feet
            bt_aprs.spd = AIRBRAKE.getVelocity().z();
            bt_aprs.hdg = AIRBRAKE.getHeading();
            Vector<3> euler = AIRBRAKE.getOrientation().toEuler();
            bt_aprs.orient[0] = euler.x();
            bt_aprs.orient[1] = euler.y();
            bt_aprs.orient[2] = euler.z();
            bt_aprs.stateFlags.setEncoding(encoding, 3);

            uint8_t arr[] = {(uint8_t)(int)AIRBRAKE.actualAngle, (uint8_t)AIRBRAKE.getStage(), (uint8_t)AIRBRAKE.estimated_apogee};
            aprs.stateFlags.pack(arr);
            bt_msg.encode(&bt_aprs);
            
            btTransmitter.send(bt_aprs);
        }
    }

#ifdef TEST_WITH_SERIAL
    if (loop)
    {
        Serial.printf("[][],%d\n", AIRBRAKE.stepToDegree(AIRBRAKE.desiredStep)); // Used for only software testing
        // Serial.printf("[][],%d\n", AIRBRAKE.stepToDegree(enc.getSteps())); // Used for encoder in the loop testing
    }
#endif
}
