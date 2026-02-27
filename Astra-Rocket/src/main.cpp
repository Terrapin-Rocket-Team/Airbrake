#include <Arduino.h>

#include <AstraRocket.h>
#include <Sensors/HW/GPS/SAM_M10Q.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Sensors/VoltageSensor/VoltageSensor.h>
#include <RecordData/Logging/DataLogger.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>

#include <cstdlib>
#include <cstring>

#include "AirbrakeController.h"

#ifdef NATIVE
#include <MotorDriver/MDNative.h>
#else
#include <MotorDriver/MDODrive.h>
#endif

#include "RuntimeHelpers.h"
#include "MessageHandlers.h"
#include <Sensors/HW/Mag/LIS3MDL.h>

using namespace astra;
using namespace astra_rocket;

static AstraRocketConfig config;
AstraRocket rocket(config);

#ifdef NATIVE
static MDNative motorDriver("MotorDriver");
#else
static MDODrive motorDriver("MotorDriver", Serial1, 35, 36);
#endif

static AirbrakeController airbrakeCtrl(&motorDriver, nullptr, nullptr, "AirbrakeCtrl");

static VoltageSensor voltageSens(A0, 787, 1000, "Bat Voltage");
static BMI088 imu;
static DPS368 baro;
static astra::LIS3MDL mag;
static SAM_M10Q gps;

static bool hitlReadyForData = false;

static const uint32_t STATIONARY_CAL_TIME_MS = 3000; // 3s still
static const uint32_t MAG_CAL_TIME_MS = 30000;       // 30s rotate

#ifdef ENV_TEENSY
static void printTeensyCrashReport()
{
    if (CrashReport)
    {
        Serial.println("=== Previous Crash Report ===");
        Serial.print(CrashReport);
        Serial.println("=== End Crash Report ===");
        CrashReport.clear();
    }
}
#endif

static void emitFullTelemHeader(Stream &out)
{
    PrintLog tempLog(out, true);
    if (!tempLog.begin())
        return;
    if (DataLogger::available())
    {
        DataLogger::instance().printHeaderTo(&tempLog);
    }
    tempLog.end();
}

void setup()
{
    Serial.begin(115200);
#if defined(NATIVE)
    config.withHITL(true);
    Serial.println("SITL mode enabled");
#else
    delay(2000);

    Serial2.begin(115200); // Radio/Radxa serial port
    printTeensyCrashReport();

    //
    // config.withHITL(true);
    //

    config.with6DoFIMU(&imu)
        .withMag(&mag)
        .withBaro(&baro)
        .withGPS(&gps);

    //imu.setMountingOrientation(MountingOrientation::FLIP_XZ); // Adjust based on your mounting
    mag.setMountingOrientation(MountingOrientation::ROTATE_180_Z);
    // Poll mag below its default ODR to avoid repeated identical samples tripping stuck-reading health checks.
    mag.setUpdateRate(20);
#endif

    config.withMiscSensor(&motorDriver).withMiscSensor(&voltageSens);
    config.withBaroMachLockout(true, 0.7);

    if (!rocket.init())
    {
        LOGE("ASTRA FAILED TO INIT");
    }

    // Configure Mahony gains only after state/filter exist.
    auto *rocketState = rocket.getRocketState();
    auto *orifilter = rocketState ? rocketState->getOrientationFilter() : nullptr;
    if (orifilter)
    {
        orifilter->setKp(0.8);
        orifilter->setKi(0.001);
    }

    // runtime_helpers::runOrientationCalibration(rocket,
    //                                            config,
    //                                            usingHitlSensors,
    //                                            Serial,
    //                                            STATIONARY_CAL_TIME_MS,
    //                                            MAG_CAL_TIME_MS);

    if (motorDriver.isInitialized())
    {
        motorDriver.zeroMotor();
    }
    else
    {
        LOGE("Motor Not Initialized!");
    }

    airbrakeCtrl.setRocketState(rocket.getRocketState());
    airbrakeCtrl.installBaroWrapper(config.getSensorManager());
    airbrakeCtrl.begin();
    airbrakeCtrl.setTargetApogee(1300.0);
    airbrakeCtrl.setBinarySearchParams(10, 10.0, 5.0);
    airbrakeCtrl.setAngleLimits(0.0f, motorDriver.getMaxAngle());
    airbrakeCtrl.setRocketParameters(21.0, 0.01168, 0.00987); // mass kg, CdA of rocket m^2 , flap area m^2
    airbrakeCtrl.setGroundAltitude(137.0);                    // m
    airbrakeCtrl.setTransonicLockout(true, 0.7);
    airbrakeCtrl.setSimulationParams(0.05, 45.0);         // sim for apogee prediction
    airbrakeCtrl.enableAdaptiveCdA(true, 0.2);            // ??
    airbrakeCtrl.enableBaroCorrection(true, 0.052, 0.15); // correction c, tau
    airbrakeCtrl.enable();

    Astra *astraSys = rocket.getAstraSystem();
    if (astraSys && astraSys->getMessageRouter())
    {
        astraSys->getMessageRouter()->withListener("AB/", [](const char *msg, const char *prefix, Stream *src)
                                                   { handleAirbrakeMessage(msg, prefix, src, airbrakeCtrl, motorDriver); });
        astraSys->getMessageRouter()->withListener("HITL/", [](const char *msg, const char *prefix, Stream *src)
                                                   { handleHitlMessage(msg, prefix, src, hitlReadyForData); });
        Serial.println("AB commands enabled: AB/TARGET_APOGEE <m>, AB/ANGLE <deg>");
    }
    else
    {
        Serial.println("AB router unavailable");
    }
}

void loop()
{
    rocket.update();

    if (!hitlReadyForData && motorDriver.isInitialized())
    {
        hitlReadyForData = true;
        Serial.println("HITL READY");
    }
    // In PAD_IDLE, do nothing.
    if (rocket.getRocketState()->getFlightStage() == PAD_IDLE)
    {
        return;
    }
    airbrakeCtrl.update();
}
