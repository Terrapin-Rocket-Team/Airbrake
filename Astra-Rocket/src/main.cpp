#include <Arduino.h>

#include <Utils/Astra.h>
#ifndef NATIVE
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Mag/LIS3MDL.h>
#include <Sensors/HW/Baro/DPS368.h>
#endif
#include <Sensors/VoltageSensor/VoltageSensor.h>
#include <RecordData/Logging/DataLogger.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>
#include <Filters/DefaultKalmanFilter.h>
#include <Filters/Mahony.h>
#include <RocketState.h>

#include "AirbrakeController.h"

#ifdef NATIVE
#include <MotorDriver/MDNative.h>
#else
#include <MotorDriver/MDODrive.h>
#endif

#include "RuntimeHelpers.h"
#include "MessageHandlers.h"

using namespace astra;

static DefaultKalmanFilter rocketKalmanFilter;
static MahonyAHRS rocketOrientationFilter(0.1, 0.0005);
astra_rocket::RocketState rocketState(&rocketKalmanFilter, &rocketOrientationFilter);

static AstraConfig config;
Astra rocket(&config);

#ifdef NATIVE
static MDNative motorDriver("MotorDriver");
#else
static MDODrive motorDriver("MotorDriver", Serial1, 35, 34);
#endif
static AirbrakeController airbrakeController(&motorDriver, &rocketState);

#ifndef NATIVE
static BMI088 imu;
static DPS368 baro;
static astra::LIS3MDL mag;
#endif

static PrintLog serialLog(Serial, true);
static PrintLog radioLog(Serial2, true);

#ifdef ENV_TEENSY
static FileLogSink fileDLog("data_log.txt", StorageBackend::SD_CARD, false);
static FileLogSink fileELog("event_log.txt", StorageBackend::SD_CARD, false);

static ILogSink *logSinks[] = {&serialLog, &radioLog, &fileDLog};
static ILogSink *eventSinks[] = {&serialLog, &radioLog, &fileELog};
#else
static ILogSink *logSinks[] = {&serialLog, &radioLog};
static ILogSink *eventSinks[] = {&serialLog, &radioLog};
#endif

static void printTeensyCrashReport()
{
#ifdef ENV_TEENSY
    if (CrashReport)
    {
        Serial.println("=== Previous Crash Report ===");
        Serial.print(CrashReport);
        Serial.println("=== End Crash Report ===");
        CrashReport.clear();
    }

#endif
}

void setup()
{
    Serial.begin(115200);
#ifndef NATIVE
    Serial2.begin(115200);
#endif

    // serialLog.begin();
    // radioLog.begin();
    // fileDLog.begin();
    // fileELog.begin();

    delay(2000);
    printTeensyCrashReport();

#ifdef NATIVE
    config.withState(&rocketState)
        .withReporter(&airbrakeController)
        .withReporter(&rocketState)
        .withEventLogs(eventSinks, sizeof(eventSinks) / sizeof(eventSinks[0]))
        .withDataLogs(logSinks, sizeof(logSinks) / sizeof(logSinks[0]))
        .withLoggingRate(2);
#else
    config.with6DoFIMU(&imu)
        .withMag(&mag)
        .withBaro(&baro)
        .withState(&rocketState)
        .withReporter(&airbrakeController)
        .withReporter(&rocketState)
        .withEventLogs(eventSinks, sizeof(eventSinks) / sizeof(eventSinks[0]))
        .withDataLogs(logSinks, sizeof(logSinks) / sizeof(logSinks[0]))
        .withLoggingRate(2);
#endif

#ifndef NATIVE
    mag.setMountingOrientation(MountingOrientation::FLIP_XY);
    mag.setUpdateRate(20);
#endif

    airbrakeController.setAutoUpdate(true);
    airbrakeController.begin();
    config.withMiscSensor(&motorDriver);
    config.withBaroMachLockout(true, 0.7);

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

    if (rocket.getMessageRouter())
    {
#ifdef NATIVE
        rocket.getMessageRouter()->withListener("AB/", [](const char *msg, const char *prefix, Stream *src)
                                                { handleAirbrakeMessage(msg, prefix, src, motorDriver); });
#else
        rocket.getMessageRouter()->withInterface(&Serial2);
        rocket.getMessageRouter()->withListener("AB/", [](const char *msg, const char *prefix, Stream *src)
                                                { handleAirbrakeMessage(msg, prefix, src, motorDriver); });
#endif
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
    updateAirbrakeSweep(motorDriver);
}
