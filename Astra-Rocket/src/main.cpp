#include <Arduino.h>

#include <AstraRocket.h>
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
#include "AvionicsPacketProtocol.h"
#include "Helpers/MessageHandlers.h"
#include "Helpers/PacketStreams.h"
#include "Telemetry/AirbrakeTelemetry.h"

#ifdef NATIVE
#include <MotorDriver/MDNative.h>
#define A1 (0)
#else
#include <MotorDriver/MDODrive.h>
#endif

using namespace astra;
using namespace astra_rocket;

static DefaultKalmanFilter rocketKalmanFilter;
static MahonyAHRS rocketOrientationFilter(0.8, 0.001);
static AstraRocketConfig config;
static RocketState rocketState(&rocketKalmanFilter, &rocketOrientationFilter, &config);
static AstraRocket rocket(config);

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
static PacketStreams telemetryStreams;
// static AirbrakeTelemetryPublisher airbrakeTelemetry(rocketState, config, airbrakeCtrl, voltageSens, motorDriver, telemetryStreams);
static FileLogSink fileDLog("data_log.txt", StorageBackend::SD_CARD, false);
static FileLogSink fileELog("event_log.txt", StorageBackend::SD_CARD, false);
static ILogSink *logSinks[] = {&fileDLog, &serialLog};
static ILogSink *eventSinks[] = {&serialLog, &fileELog};
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
    else
    {
        Serial.println("No crash report from previous run.");
    }
}
#endif

#include "RadioMessage.h"

#define SERIAL_BAUD 115200 // bits/s

#define END_CHAR '\n'

HardwareSerial *telemSer = (HardwareSerial *)&Serial;

uint32_t telemTimer = millis();

const uint32_t telemInterval = 100; // ms -> 10 Hz

Message m;

APRSConfig aprscfg = {"KD3BBD", "ALL", "WIDE1-1", PositionWithoutTimestampWithoutAPRS, '\\', 'M'};

APRSTelem telem(aprscfg);

// uint8_t stflEncoding[] = {7, 4, 5}; // (Avionics): Temp, stage, fix qual

uint8_t stflEncoding[] = {7, 5, 8, 7, 4}; // (Airbrake): Temp, flap angle, pred apogee (x2), stage

void setup()
{
    Serial.begin(115200);
#ifndef NATIVE
#ifdef ENV_TEENSY
    // setupAirbrakeTelemetrySerial(telemetryStreams, Serial2);
    // Serial2.println("hello world");
    telemSer->begin(SERIAL_BAUD);

    telem.stateFlags.setEncoding(stflEncoding, sizeof(stflEncoding));
#else
    Serial2.begin(115200);
#endif
    delay(2000);
#ifdef ENV_TEENSY
    printTeensyCrashReport();
#endif
#endif
    config.withPreflightLogRate(20);
    config.withFlightLogRate(20);
    config.withPostflightLogRate(20);

    config.withState(&rocketState)
        .withReporter(&airbrakeCtrl)
        .withMiscSensor(&motorDriver)
        .withMiscSensor(&voltageSens)
        .withEventLogs(eventSinks, sizeof(eventSinks) / sizeof(eventSinks[0]))
        .withDataLogs(logSinks, sizeof(logSinks) / sizeof(logSinks[0]))

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
        //.withHITL()  ////change this!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        .withHITLInterface(&Serial);
    Serial2.println("Sensors configured");
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
    if (!airbrakeCtrl.installBaroWrapper(config.getSensorManager()))
    {
        LOGE("Failed to install airbrake baro wrapper. FC-side baro correction will be inactive.");
    }
    airbrakeCtrl.begin();
    airbrakeCtrl.setTargetApogee(8382.0);
    airbrakeCtrl.setBinarySearchParams(10, 1, 1);
    airbrakeCtrl.setAngleLimits(0.0f, motorDriver.getMaxAngle());
    airbrakeCtrl.setRocketParameters((90.98) / 2.2, 0.47, 0.00987); // dry mass kg, Cd of rocket , flap area m^2
    airbrakeCtrl.setGroundAltitude(912.0);                          // m
    airbrakeCtrl.setTransonicLockout(true, 0.7);
    airbrakeCtrl.setSimulationParams(0.05, 45.0);          // sim for apogee prediction
    airbrakeCtrl.enableAdaptiveCdA(false, 0.2);            // ??
    airbrakeCtrl.enableBaroCorrection(true, 0.0489, 0.15); // correction c, tau (unused)
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
#ifdef ENV_TEENSY
    // airbrakeTelemetry.publishIfDue();
#endif

    if (millis() - telemTimer > telemInterval)
    {

        telemTimer = millis();
        // Kloudbusters: 37°10'06.2"N 97°44'17.8"W
        telem.lat = 37.168389;                                                 // decimal latitude
        telem.lng = 97.738278;                                                 // decimal longitude
        telem.alt = airbrakeCtrl.state->getPosition().z();                     // ft
        telem.spd = airbrakeCtrl.state->getVelocity().magnitude() * 0.5144444; // knots (converted from m/s)
        telem.hdg = airbrakeCtrl.state->getHeading();                          // degree
        auto orient = airbrakeCtrl.state->getOrientation().toEuler321();
        telem.orient[0] = orient[0]; // euler angles in degrees (x)
        telem.orient[1] = orient[1]; // euler angles in degrees (y)
        telem.orient[2] = orient[2]; // euler angles in degrees (z)

        // Avionics
        // uint8_t temp = 0; // deg C
        // uint8_t stage = 0; // #
        // uint8_t fixQual = 0; // #
        // uint8_t flags[] = {temp, stage, fixQual};
        // telem.stateFlags.set(flags);

        // Airbrake
        uint16_t predApogee = airbrakeCtrl.getPredictedApogee(); // ft
        uint8_t temp = baro.getTemp();                           // deg C
        uint8_t flapAng = airbrakeCtrl.getCurrentDeployment();   // deg
        uint8_t predApogee1 = predApogee >> 8;                   // (DONT CHANGE)
        uint8_t predApogee2 = predApogee & 0x00ff;               // (DONT CHANGE)
        uint8_t stage = airbrakeCtrl.state->getFlightStage();    // #
        uint8_t flags[] = {temp, flapAng, predApogee1, predApogee2, stage};
        telem.stateFlags.set(flags);

        m.encode(&telem)->print(*telemSer); // automatically terminates with \n
    }
}
