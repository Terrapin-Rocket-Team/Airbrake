#include <Arduino.h>

#include "airbrake_state.h"
#include "vn_100.h"
#include "AirbrakeKF.h"
#include "e5.h"
#include "BR.h"

// Buzzer
const int BUZZER_PIN = 23;
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

// Encoder pins
const int enc_chan_a = 36;
const int enc_chan_b = 37;

// Sensors
//E5 enc(enc_chan_a, enc_chan_b, "E5"); // Encoder
VN_100 vn(&SPI, 10); // Vector Nav
mmfs::DPS310 baro1; // Avionics Sensor Board 1.1
mmfs::MS5611 baro2; // Avionics Sensor Board 1.1
mmfs::BMI088andLIS3MDL airbrake_imu; // Avionics Sensor Board 1.1
mmfs::MAX_M10S gps; // Avionics Sensor Board 1.1
BR blueRaven;
mmfs::Sensor* airbrake_sensors[6] = {&baro1, &baro2, &airbrake_imu, &gps, &vn, &blueRaven};

// Initialize Airbrake State
AirbrakeKF kf;
AirbrakeState AIRBRAKE(airbrake_sensors, 6, &kf, BUZZER_PIN);

// MMFS Stuff
mmfs::Logger logger(120, 5);
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

void setup() {
    // Initialize Serial and SPI Buses
    Serial.begin(115200);
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.setSCK(13);
    SPI.begin();

    if (CrashReport) Serial.println(CrashReport);

    // Immediately turn the motor off (needs the break pin set to high)
    pinMode(brk_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    digitalWrite(brk_pin, HIGH);
    digitalWrite(dir_pin, LOW);

    // MMFS Stuff
    SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
    SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;
    psram = new mmfs::PSRAM();
    logger.init(&AIRBRAKE);

    logger.recordLogData(mmfs::INFO_, "Entering Setup");

    // Check the sd card
    if (!(logger.isSdCardReady())){
        logger.recordLogData(mmfs::INFO_, "SD Card Failed to Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else{
        bb.onoff(BUZZER_PIN, 1000, 1);
    }

    // Check the psram
    if (!(logger.isPsramReady())){
        logger.recordLogData(mmfs::INFO_, "PSRAM Failed to Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else {
        bb.onoff(BUZZER_PIN, 1000, 1);
    } 
    
    // Initialize State (runs Begin/Init for each sensor)
    if(!AIRBRAKE.init()){
        logger.recordLogData(mmfs::INFO_, "State Failed to Completely Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else{ 
        bb.onoff(BUZZER_PIN, 1000, 1);
        baro1.setBiasCorrectionMode(true);
        baro2.setBiasCorrectionMode(true);
        gps.setBiasCorrectionMode(true);
    }

    logger.writeCsvHeader();
    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
    

    
    

}

//static double last = 0; // for better timing than "delay(100)"
void loop() {

    bb.update();
    static unsigned long lastUpdateTime = 0;
    const unsigned long UPDATE_INTERVAL = 100; // 100ms between updates

    if (millis() - lastUpdateTime < UPDATE_INTERVAL) {
        return;
    }
    lastUpdateTime = millis();

    Serial.println("--- Blue Raven Update ---");

    // Update Blue Raven
    blueRaven.update();

    // Check connection status
    Serial.print("Connection status: ");
    if (blueRaven.isDeviceConnected()) {
        Serial.println("Connected");
    } else {
        Serial.println("Disconnected");
        //return;
    }

    // Print raw buffer availability, this check is for hardware, see if the teensy is getting any data
    Serial.print("Raw buffer available: ");
    Serial.println(blueRaven.getAvailableBytes());

    // First way to print sensor data
    
    
    Serial.println("Sensor Readings:");
    Serial.print("Altitude: "); Serial.print(blueRaven.getAltitude()); Serial.println(" ft");
    Serial.print("Pressure: "); Serial.print(blueRaven.getPressure()); Serial.println(" Pa");
    Serial.print("Temperature: "); Serial.print(blueRaven.getTemperature()); Serial.println(" C");
    Serial.print("Velocity: "); Serial.print(blueRaven.getVelocity()); Serial.println(" m/s");
    Serial.print("Tilt Angle: "); Serial.print(blueRaven.getTiltAngle()); Serial.println(" deg");
    Serial.print("Roll Angle: "); Serial.print(blueRaven.getRollAngle()); Serial.println(" deg");

    
   
   // Second way to print sensor data
   /*
    Serial.print(blueRaven.getPressure());
    Serial.print("\t");
    Serial.print(blueRaven.getTemperature());
    Serial.print("\t");
    Serial.print(blueRaven.getAltitude());
    Serial.print("\t");
    Serial.print(blueRaven.getVelocity());
    Serial.print("\t");
    Serial.print(blueRaven.getAccelerationX());
    Serial.print("\t");
    Serial.print(blueRaven.getAccelerationY());
    Serial.print("\t");
    Serial.print(blueRaven.getAccelerationZ());
    Serial.print("\t");
    Serial.print(blueRaven.getGyroX());
    Serial.print("\t");
    Serial.print(blueRaven.getGyroY());
    Serial.print("\t");
    Serial.print(blueRaven.getGyroZ());
    Serial.print("\t");
    Serial.print(blueRaven.getRollAngle());
    Serial.print("\t");
    Serial.println(blueRaven.getTiltAngle());
    */

    // Update state and log data
    AIRBRAKE.updateState();
    logger.recordFlightData();

    // Additional state checks and actions
    if (AIRBRAKE.stage == BOOST) {
        baro1.setBiasCorrectionMode(false);
        baro2.setBiasCorrectionMode(false);
        gps.setBiasCorrectionMode(false);
    }

    Serial.println("--- End of Update ---\n");
}

