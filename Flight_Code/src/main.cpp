#include <Arduino.h>

#include "airbrake_state.h"
#include "AirbrakeKF.h"
#include <Utils/Astra.h>
#include "RetrieveData/SerialHandler.h"
#include <Math/Vector.h>
#include <Math/Quaternion.h>
#include "md6.h"
#include <Sensors/Baro/DPS368.h>
#include <Sensors/IMU/BMI088andLIS3MDL.h>
#include <Sensors/GPS/MAX_M10S.h>


// Buzzer
const int BUZZER_PIN = 23;

// Motor Driver
astra::MotorDriver mot("MotorDriver");

//Setting which senseors are used mock/real

//real sensors for flight
astra::DPS368 baro1; // Avionics Sensor Board 1.2
astra::BMI088andLIS3MDL airbrake_imu; // Avionics Sensor Board 1.2
astra::MAX_M10S gps;                  // Avionics Sensor Board 1.2

astra::Sensor *airbrake_sensors[4] = {&baro1, &airbrake_imu, &gps, &mot};


// // Initialize Airbrake State
AirbrakeKF lkfmm;
AirbrakeState AIRBRAKE(airbrake_sensors, sizeof(airbrake_sensors) / 4, &lkfmm);

// // astra Stuff
astra::AstraConfig config = astra::AstraConfig()
                              .withState(&AIRBRAKE)
                              .withBuzzerPin(BUZZER_PIN)
                              .withUpdateRate(10);

astra::Astra sys(&config);

void setup()
{
    // Initialize Serial and SPI Buses
    Serial.begin(115200);

    Serial.println("Starting Setup");
    sys.init(); // Initialize astra System
    Serial.println("Astra System Initialized");

    // Limit Switch and Motor Zeroing
    if (mot.isInitialized())
    {   
        LOGI("Zeroing Motor.");
        Serial.println("Zeroing Motor.");
        mot.zeroMotor();
        LOGI("Motor Zeroed.");
        Serial.println("Motor Zeroed.");
    }
    delay(1000);

    LOGI("Setup Complete.");
}


void loop()
{

    sys.update(); // Update astra System    

    mot.setPos(mot.angleToPos(70)); // Set motor to 10 degrees
    Serial.println("Motor Position: " + String(mot.posToAngle(mot.getPosition())));

    delay(100); // Small delay to avoid overwhelming the serial output


    // if (AIRBRAKE.stage == DEPLOY)
    // {
    //     Serial.println("Airbrake Deployed");
    //     mot.setPos(mot.angleToPos(60)); // Set motor to 60 degrees
    // }
    // else
    // {
    //     mot.setPos(mot.angleToPos(0)); // Set motor to 0 degrees
    // }
    
}

