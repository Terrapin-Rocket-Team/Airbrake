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
#include <Sensors/HW/GPS/SAM_M10Q.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Mag/MMC5603NJ.h>
#include <Sensors/HW/Baro/MS5611.h>
#include <Sensors/VoltageSensor/VoltageSensor.h>
#include <Sensors/SensorManager/SensorManager.h>
#include "md6.h"

using namespace astra_rocket;
using namespace astra;

MotorDriver mot("MotorDriver");
VoltageSensor vs(A0, 787, 1000, "Bat Voltage");

// Handler for AIRBRAKE/XX commands
void handleAirbrakeCommand(const char *message, const char *prefix, Stream *source)
{
  // message is just the angle value (e.g., "60" when sent as "AIRBRAKE/60")
  int angle = atoi(message);

  // Validate range (0-80 degrees based on motor specs)
  if (angle < 0 || angle > 80)
  {
    source->printf("ERROR: Angle %d out of range (0-80)\n", angle);
    LOGW("Invalid airbrake angle command: %d", angle);
    return;
  }

  // Convert to motor position and command
  float pos = mot.angleToPos(angle);
  mot.setPos(pos);

  // Send confirmation
  source->printf("OK: Airbrake set to %d degrees (pos=%.2f)\n", angle, pos);
  LOGI("Airbrake angle set to %d degrees", angle);
}

// Create sensor instances
SensorManager *sensorManager = new SensorManager();

// Create a custom configuration with LED status pins
// NOTE: config must be static because AstraRocket stores a reference to it
static AstraRocketConfig config;
AstraRocket rocket(config);

void setup()
{
  // Initialize Serial for debug output
  Serial.begin(115200);
  delay(2000); // Wait for serial connection

  // Configure sensors on SensorManager
  BMI088 *imu = new BMI088();
  sensorManager->setAccelSource(imu->getAccelSensor());
  sensorManager->setGyroSource(imu->getGyroSensor());
  sensorManager->setMagSource(new MMC5603NJ());
  sensorManager->setBaroSource(new astra::MS5611());
  sensorManager->setGPSSource(new SAM_M10Q());

  config
      .withSensorManager(sensorManager);

  if (!rocket.init())
  {
    Serial.println("ERROR: AstraRocket initialization failed!");
    LOGE("ASTRA FAILED TO INIT");
  }

  // Register AIRBRAKE handler with Astra's built-in message router
  if (rocket.getAstraSystem() && rocket.getAstraSystem()->getMessageRouter())
  {
    rocket.getAstraSystem()->getMessageRouter()->withListener("AIRBRAKE/", handleAirbrakeCommand);
    LOGI("AIRBRAKE command handler registered with Astra message router");
  }

  if (mot.begin())
  {
    mot.zeroMotor();
  }
  else
  {
    LOGE("Motor Not Initialized");
  }
  vs.begin();
}

void loop()
{
  FlightStage f = rocket.getRocketState()->getFlightStage();
  vs.update();
  mot.read();
  rocket.update(); // This also updates Astra's internal message router
  if (f == rocket.getRocketState()->getFlightStage())
    return;
  if (rocket.getRocketState()->getFlightStage() == FlightStage::COAST)
  {
    mot.setPos(mot.angleToPos(60));
  }
  else
  {
    mot.setPos(mot.angleToPos(0));
  }
}
