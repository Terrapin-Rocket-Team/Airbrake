#include <Arduino.h>

#include <AstraRocket.h>
#include <Sensors/HW/GPS/SAM_M10Q.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/MS5611.h>
#include <Sensors/HW/Mag/MMC5603NJ.h>
#include <Sensors/VoltageSensor/VoltageSensor.h>

#include "AirbrakeController.h"
#include "md6.h"

using namespace astra;
using namespace astra_rocket;

static AstraRocketConfig config;
AstraRocket rocket(config);

MotorDriver mot("MotorDriver");
VoltageSensor vs(A0, 787, 1000, "Bat Voltage");
astra::MS5611 rawBaro("MS5611");
AirbrakeController airbrakeCtrl(&mot, nullptr, nullptr, "AirbrakeCtrl");

void setup()
{
    Serial.begin(115200);
#if defined(NATIVE)
    config.withHITL(true);
    Serial.println("SITL mode enabled");
    if (!Serial.connectSITL("localhost", 5555))
    {
        Serial.println("ERROR: Failed to connect to SITL server");
    }
#else
    delay(2000);

    BMI088 *imu = new BMI088();
    config.with6DoFIMU(imu)
        .withMag(new MMC5603NJ())
        .withBaro(&rawBaro)
        .withGPS(new SAM_M10Q());
#endif

    config.withMiscSensor(&mot).withMiscSensor(&vs);
    config.withBaroMachLockout(true, 0.75);

    if (!rocket.init())
    {
        Serial.println("ERROR: AstraRocket initialization failed!");
        LOGE("ASTRA FAILED TO INIT");
    }

    if (mot.isInitialized())
    {
        mot.zeroMotor();
    }
    else
    {
        LOGE("Motor Not Initialized");
    }

    airbrakeCtrl.setRocketState(rocket.getRocketState());
    airbrakeCtrl.installBaroWrapper(config.getSensorManager());
    airbrakeCtrl.begin();
    airbrakeCtrl.setTargetApogee(9144.0);
    airbrakeCtrl.setBinarySearchParams(10, 10.0, 5.0);
    airbrakeCtrl.setAngleLimits(0.0, 65.0);
    airbrakeCtrl.setRocketParameters(43.5, 0.01168, 0.00987);
    airbrakeCtrl.setGroundAltitude(884.0);
    airbrakeCtrl.setTransonicLockout(true, 0.7);
    airbrakeCtrl.setSimulationParams(0.05, 45.0);
    airbrakeCtrl.enableAdaptiveCdA(true, 0.2);
    airbrakeCtrl.enableBaroCorrection(true, 0.052, 0.15);
    airbrakeCtrl.enable();
}

void loop()
{
    rocket.update();
    airbrakeCtrl.update();
}
