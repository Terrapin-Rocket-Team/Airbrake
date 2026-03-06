#include <Arduino.h>

#include <Utils/Astra.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Mag/LIS3MDL.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Sensors/VoltageSensor/VoltageSensor.h>
#include <RecordData/DataReporter/SimpleDataReporter.h>
#include <RecordData/Logging/DataLogger.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>
#include <State/DefaultState.h>

#include "AirbrakeController.h"

#ifdef NATIVE
#include <MotorDriver/MDNative.h>
#else
#include <MotorDriver/MDODrive.h>
#endif

#include "RuntimeHelpers.h"
#include "MessageHandlers.h"

using namespace astra;

DefaultState rocketState;

static AstraConfig config;
Astra rocket(&config);

#ifdef NATIVE
static MDNative motorDriver("MotorDriver");
#else
static MDODrive motorDriver("MotorDriver", Serial1, 35, 34);
#endif

static BMI088 imu;
static DPS368 baro;
static astra::LIS3MDL mag;


static PrintLog serialLog(Serial, true);
static PrintLog radioLog(Serial2, true);

static FileLogSink fileDLog("data_log.txt", StorageBackend::SD_CARD, false);
static FileLogSink fileELog("event_log.txt", StorageBackend::SD_CARD, false);

static ILogSink *logSinks[] = { &serialLog, &radioLog, &fileDLog };
static ILogSink *eventSinks[] = { &serialLog, &radioLog, &fileELog };

static bool beginMillisReporter()
{
    return true;
}

static float updateMillisReporter()
{
    return millis() / 1000.0;
}

static SimpleDataReporter<float> millisReporter(
    "Time",
    "%0.3f",
    "Seconds",
    beginMillisReporter,
    updateMillisReporter,
    0u);

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
    Serial2.begin(115200);

    // serialLog.begin();
    // radioLog.begin();
    // fileDLog.begin();
    // fileELog.begin();
    millisReporter.begin();
    
    delay(2000);
    printTeensyCrashReport();

    DataLogger::unregisterReporter(&imu);
    DataLogger::unregisterReporter(&mag);
    DataLogger::unregisterReporter(&rocketState);
    DataLogger::registerReporter(&millisReporter);
    DataLogger::registerReporter(&baro);
    DataLogger::registerReporter(&motorDriver);
    Serial.printf("DL reporters pre-init: %u\n", DataLogger::instance().getNumReporters());

    config.with6DoFIMU(&imu)
        .withMag(&mag)
        .withBaro(&baro)
        .withState(&rocketState)
        .withEventLogs(eventSinks, sizeof(eventSinks) / sizeof(eventSinks[0]))
        .withDataLogs(logSinks, sizeof(logSinks) / sizeof(logSinks[0]))
        .withLoggingRate(2);
        ;

    mag.setMountingOrientation(MountingOrientation::FLIP_XY);
    mag.setUpdateRate(20);

    config.withMiscSensor(&motorDriver);
    config.withBaroMachLockout(true, 0.7);

    if (!rocket.init())
    {
        LOGE("ASTRA FAILED TO INIT");
    }
    Serial.printf("DL reporters post-init: %u (available=%d)\n",
                  DataLogger::instance().getNumReporters(),
                  DataLogger::available() ? 1 : 0);
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
        rocket.getMessageRouter()->withInterface(&Serial2);
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
    updateAirbrakeSweep(motorDriver);
}
