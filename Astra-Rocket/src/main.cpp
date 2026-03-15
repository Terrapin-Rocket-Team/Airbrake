#include <Arduino.h>

#include <Utils/Astra.h>
#include <AstraRocketConfig.h>
#include <Filters/DefaultKalmanFilter.h>
#include <Filters/Mahony.h>
#include <RocketState.h>
#ifndef NATIVE
#include <Sensors/HW/Baro/DPS368.h>
#include <Sensors/HW/GPS/SAM_M10Q.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Mag/LIS3MDL.h>
#endif
#include <Sensors/VoltageSensor/VoltageSensor.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>

#include "AirbrakeController.h"

#ifdef NATIVE
#include <MotorDriver/MDNative.h>
#else
#include <MotorDriver/MDODrive.h>
#endif

#include "MessageHandlers.h"

using namespace astra;
using namespace astra_rocket;

static DefaultKalmanFilter rocketKalmanFilter;
static MahonyAHRS rocketOrientationFilter(0.8, 0.001);
static AstraRocketConfig config;
static RocketState rocketState(&rocketKalmanFilter, &rocketOrientationFilter, &config);
static Astra rocket(&config);

#ifdef NATIVE
static MDNative motorDriver("MotorDriver");
#else
static MDODrive motorDriver("MotorDriver", Serial1, 35, 34);
#endif

static AirbrakeController airbrakeCtrl(&motorDriver, &rocketState, nullptr, "AirbrakeCtrl");
#ifdef NATIVE
static VoltageSensor voltageSens(A0, 22000, 33000, "Bat Voltage");
#else
static VoltageSensor voltageSens(A1, 22000, 33000, "Bat Voltage");
#endif

#ifndef NATIVE
static BMI088 imu;
static DPS368 baro;
static astra::LIS3MDL mag;
static SAM_M10Q gps;
#endif

static PrintLog serialLog(Serial, true);

#ifdef ENV_TEENSY
static PrintLog radioLog(Serial2, true);
static FileLogSink fileDLog("data_log.txt", StorageBackend::SD_CARD, false);
static FileLogSink fileELog("event_log.txt", StorageBackend::SD_CARD, false);
static ILogSink *logSinks[] = {&serialLog, &radioLog, &fileDLog};
static ILogSink *eventSinks[] = {&serialLog, &radioLog, &fileELog};
#else
static ILogSink *logSinks[] = {&serialLog};
static ILogSink *eventSinks[] = {&serialLog};
#endif

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

void setup()
{
    Serial.begin(115200);
#ifndef NATIVE
    Serial2.begin(115200);
    delay(2000);
#ifdef ENV_TEENSY
    printTeensyCrashReport();
#endif
#endif

    config.withState(&rocketState)
        .withReporter(&airbrakeCtrl)
        .withMiscSensor(&motorDriver)
        .withMiscSensor(&voltageSens)
        .withEventLogs(eventSinks, sizeof(eventSinks) / sizeof(eventSinks[0]))
        .withDataLogs(logSinks, sizeof(logSinks) / sizeof(logSinks[0]))
        .withLoggingRate(2)
        .withBaroMachLockout(true, 0.7);

#ifndef NATIVE
    config.with6DoFIMU(&imu)
        .withMag(&mag)
        .withBaro(&baro)
        .withGPS(&gps);

    mag.setMountingOrientation(MountingOrientation::FLIP_XY);
    mag.setUpdateRate(20);
#endif

    const int initResult = rocket.init();
    if (initResult < 0)
    {
        Serial.println("Astra init failed");
        return;
    }

    if (motorDriver.isInitialized())
    {
        motorDriver.zeroMotor();
        motorDriver.setEnabled(false);
    }
    else
    {
        LOGE("Motor Not Initialized!");
    }

    airbrakeCtrl.setAutoUpdate(false);
    // airbrakeCtrl.installBaroWrapper(config.getSensorManager());
    airbrakeCtrl.begin();
    airbrakeCtrl.setTargetApogee(1144.0);
    airbrakeCtrl.setBinarySearchParams(10, .1, 5.0);
    airbrakeCtrl.setAngleLimits(0.0f, motorDriver.getMaxAngle());
    airbrakeCtrl.setRocketParameters(75 / 2.2, 0.01168, 0.00987);
    airbrakeCtrl.setGroundAltitude(137.0);
    airbrakeCtrl.setTransonicLockout(true, 0.7);
    airbrakeCtrl.setSimulationParams(0.05, 45.0);
    airbrakeCtrl.enableAdaptiveCdA(false, 0.2);
    airbrakeCtrl.enableBaroCorrection(false, 0.052, 0.15);
    airbrakeCtrl.enable();

    if (rocket.getMessageRouter())
    {
#ifndef NATIVE
        rocket.getMessageRouter()->withInterface(&Serial2);
#endif
        rocket.getMessageRouter()->withListener("AB/", [](const char *msg, const char *prefix, Stream *src)
                                                { handleAirbrakeMessage(msg, prefix, src, motorDriver); });
        Serial.println("AB commands enabled: AB/ANGLE <deg>, AB/SWEEP <seconds>, AB/SWEEP_STOP, AB/CRASH");
    }
    else
    {
        Serial.println("AB router unavailable");
    }
}

void loop()
{
    rocket.update();
    airbrakeCtrl.update();
    updateAirbrakeSweep(motorDriver);
}
