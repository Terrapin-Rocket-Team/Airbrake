/**
 * Basic Teensy 4.1 Sensor Example using AstraRocket Library
 *
 * This example demonstrates the simplest possible use of AstraRocket:
 * - Auto-detects available sensors (Barometer, GPS, IMU, high-G accelerometer)
 * - Automatically initializes all hardware
 * - Sets up logging to both Serial and SD card
 * - Tracks flight stages (pad idle, boost, coast, apogee, descent, landing)
 * - Adjusts logging rates based on flight phase
 * - Uses LED status indicators for sensor and GPS status
 *
 * AstraRocket handles all the complexity of:
 * - Sensor detection and initialization
 * - State estimation and filtering
 * - Flight stage detection
 * - Logging management
 * - Status indicator management
 *
 * LED Status Indicators:
 * - Pin 25 (Sensor Status): Solid ON = all sensors good, 2 blinks = sensor failure
 * - Pin 26 (GPS Status): Solid ON = GPS fix, 1 blink = GPS init but no fix, OFF = no GPS
 *
 * For more advanced usage with custom configuration, see the ConfigurableExample.
 */

#include <Arduino.h>
#include <AstraRocket.h>
#include <Sensors/GPS/MAX_M10S.h>
#include "md6.h"

using namespace astra_rocket;

astra::MotorDriver mot("MotorDriver");

DataReporter *others[] = {&mot};

// Create a custom configuration with LED status pins
AstraRocketConfig config = AstraRocketConfig()
                              //  .withHITL(true)
                               .withGPS(new MAX_M10S)
                               .withSensorStatusLEDPin(32)
                               .withFlightLogRate(1)
                               .withPreflightLogRate(1)
                               .withGPSStatusLEDPin(31);
AstraRocket rocket(config);

void setup()
{
  // Initialize Serial for debug output
  Serial.begin(115200);
  config.getAstraConfig()->withOtherDataReporters(others, sizeof(others) / sizeof(DataReporter*));
  delay(2000); // Wait for serial connection
  if (!rocket.init())
  {
    Serial.println("ERROR: AstraRocket initialization failed!");
    LOGE("ASTRA FAILED TO INIT");
  }
  if (mot.begin())
  {
    LOGI("Zeroing Motor...");
    mot.zeroMotor();
    LOGI("Motor Zeroed.");
  }
  else{
    LOGE("Motor Not Initialized");
  }
}

void loop()
{
  FlightStage f = rocket.getFlightStage();
  rocket.update();
  if (f == rocket.getFlightStage())
    return;
  if (rocket.getFlightStage() == FlightStage::COAST)
  {
    mot.setPos(mot.angleToPos(60));
  }
  else
  {
    mot.setPos(mot.angleToPos(0));
  }
}
