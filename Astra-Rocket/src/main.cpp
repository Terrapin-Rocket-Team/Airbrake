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
#include <BlueRaven/BGHGAccel.h>
#include <BlueRaven/BRAccel.h>
#include <BlueRaven/BRBaro.h>
#include <BlueRaven/BRIMU.h>
#include <BlueRaven/BlueRaven.h>
#endif
#include <Sensors/VoltageSensor/VoltageSensor.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>

#include "AirbrakeController.h"

#ifdef NATIVE
#include <MotorDriver/MDNative.h>
#define A1 (0)
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

static VoltageSensor voltageSens(A1, 22000, 33000, "Bat Voltage");

#ifndef NATIVE
static BMI088 imu;
static DPS368 baro;
static astra::LIS3MDL mag;
static SAM_M10Q gps;
static BlueRaven blueRaven;
static BRIMU blueRavenImu(blueRaven, "BlueRaven IMU Debug");
static BRBaro blueRavenBaro(blueRaven, "BlueRaven Baro Debug");
static BRAccel blueRavenAccel(blueRaven, "BlueRaven Accel Debug");
static BGHGAccel blueRavenHighG(blueRaven, "BlueRaven High-G Debug");
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

static bool hitlReadyAnnounced = false;

static void maybeAnnounceHitlReady()
{
    if (hitlReadyAnnounced || config.getRuntimeMode() != AstraConfig::RuntimeMode::HITL)
    {
        return;
    }

    hitlReadyAnnounced = true;
    LOGI("HITL READY");
    Serial.println("HITL READY");
    Serial.flush();
}

#ifndef NATIVE
static void printBlueRavenDebug()
{
    static uint32_t lastPrintedSampleCount = 0;
    static uint32_t lastStatusPrintMs = 0;

    const uint32_t sampleCount = blueRaven.getSampleCount();
    if (sampleCount != lastPrintedSampleCount && blueRaven.hasValidSample())
    {
        lastPrintedSampleCount = sampleCount;

        const auto lowG = blueRavenAccel.getAccel();
        const auto imuAccel = blueRavenImu.getAccel();
        const auto highG = blueRavenHighG.getAccel();
        const auto gyro = blueRavenImu.getAngVel();

        Serial.printf("BR[%lu] batt=%.3fV baro=%.2fhPa temp=%.2fC agl=%.2fm vel=%.2fm/s tilt=%.1f roll=%.1f\n",
                      static_cast<unsigned long>(sampleCount),
                      blueRaven.getBatteryVolts(),
                      blueRavenBaro.getPressure(),
                      blueRavenBaro.getTemp(),
                      blueRavenBaro.getAltitudeAglM(),
                      blueRaven.getVerticalVelocityMps(),
                      blueRaven.getTiltDeg(),
                      blueRaven.getRollDeg());
        Serial.printf("  BR lowG = [%.3f, %.3f, %.3f] m/s^2\n", lowG.x(), lowG.y(), lowG.z());
        Serial.printf("  BR imuA = [%.3f, %.3f, %.3f] m/s^2\n", imuAccel.x(), imuAccel.y(), imuAccel.z());
        Serial.printf("  BR highG= [%.3f, %.3f, %.3f] m/s^2\n", highG.x(), highG.y(), highG.z());
        Serial.printf("  BR gyro = [%.3f, %.3f, %.3f] rad/s\n", gyro.x(), gyro.y(), gyro.z());
        return;
    }

    const uint32_t now = millis();
    if ((now - lastStatusPrintMs) >= 2000)
    {
        lastStatusPrintMs = now;
        Serial.printf("BR status: connected=%d valid=%d samples=%lu\n",
                      blueRaven.isConnected() ? 1 : 0,
                      blueRaven.hasValidSample() ? 1 : 0,
                      static_cast<unsigned long>(sampleCount));
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
        .withGPS(&gps)
        .withMiscSensor(&blueRaven)
        .withMiscSensor(&blueRavenImu)
        .withMiscSensor(&blueRavenBaro)
        .withMiscSensor(&blueRavenAccel)
        .withMiscSensor(&blueRavenHighG)
        .withHITL()
        .withHITLInterface(&Serial)
        ;

    blueRaven.useUsbHost();
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
    airbrakeCtrl.setTargetApogee(8300.0);
    airbrakeCtrl.setBinarySearchParams(10, .1, 1);
    airbrakeCtrl.setAngleLimits(0.0f, motorDriver.getMaxAngle());
    airbrakeCtrl.setRocketParameters((120 - 37.47) / 2.2, 0.01168, 0.00987); // dry mass kg, CdA of rocket m^2 , flap area m^2
    airbrakeCtrl.setGroundAltitude(912.0);                                   // m
    airbrakeCtrl.setTransonicLockout(true, 0.7);
    airbrakeCtrl.setSimulationParams(0.05, 45.0);          // sim for apogee prediction
    airbrakeCtrl.enableAdaptiveCdA(false, 0.2);            // ??
    airbrakeCtrl.enableBaroCorrection(false, 0.052, 0.15); // correction c, tau
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
    maybeAnnounceHitlReady();
#ifndef NATIVE
    // printBlueRavenDebug();
#endif
}
