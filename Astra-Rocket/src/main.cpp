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
#include "AvionicsPacketProtocol.h"
#include "PacketStreams.h"

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
static PacketStreams telemetryStreams;
static FileLogSink fileDLog("data_log.txt", StorageBackend::SD_CARD, false);
static FileLogSink fileELog("event_log.txt", StorageBackend::SD_CARD, false);
static ILogSink *logSinks[] = { &fileDLog};
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
    else {
        Serial.println("No crash report from previous run.");
    }
}
#endif

#ifdef ENV_TEENSY
namespace
{
constexpr uint32_t kTelemetryBaud = 115200;
constexpr uint32_t kAbTelemPeriodMs = 500;

uint32_t lastAbTelemMs = 0;
bool reportedAbTelemOnline = false;

float metersToFeet(double meters)
{
    return static_cast<float>(meters * 3.28083989501312);
}

bool buildAbTelemetryPacket(avionics_packet::PacketBuffer &packet)
{
    RocketState *state = &rocketState;
    if (state == nullptr)
        return false;

    avionics_packet::AbTelemetry telemetry = {};

    telemetry.positionZFeet = metersToFeet(state->getAltitudeAGL());

    const Vector<3> velocity = state->getVelocity();
    telemetry.velocityZMs = static_cast<float>(velocity.z());

    const Vector<3> acceleration = state->getAcceleration();
    telemetry.accelZMs2 = static_cast<float>(acceleration.z());

    if (Barometer *baroSource = config.getSensorManager()->getBaroSource();
        baroSource != nullptr && baroSource->isInitialized())
    {
        telemetry.hasBaroAgl = true;
        telemetry.baroAglFeet = metersToFeet(baroSource->getASLAltM() - state->getGroundLevelMSL());
    }

    const Quaternion orientation = state->getRocketOrientation();
    telemetry.quatW = static_cast<float>(orientation.w());
    telemetry.quatX = static_cast<float>(orientation.x());
    telemetry.quatY = static_cast<float>(orientation.y());
    telemetry.quatZ = static_cast<float>(orientation.z());

    telemetry.hasBattery = voltageSens.isInitialized();
    telemetry.batteryVolts = static_cast<float>(voltageSens.getVoltage());

    telemetry.hasMotorBattery = motorDriver.isInitialized();
    telemetry.motorBatteryVolts = motorDriver.getBatVoltage();

    telemetry.desiredAngleDeg = static_cast<float>(airbrakeCtrl.getCommandedDeployment());
    telemetry.actualAngleDeg = static_cast<float>(airbrakeCtrl.getCurrentDeployment());
    telemetry.predictedApogeeFeet = metersToFeet(airbrakeCtrl.getPredictedApogee());

    return avionics_packet::encodeAbTelemetry(telemetry, packet);
}

void publishAbTelemetryIfDue()
{
    const uint32_t now = millis();
    if ((now - lastAbTelemMs) < kAbTelemPeriodMs)
        return;

    lastAbTelemMs = now;

    avionics_packet::PacketBuffer packet;
    if (!buildAbTelemetryPacket(packet))
        return;

    const bool sent = telemetryStreams.send(packet);
    if (!sent)
    {
        LOGW("ABTELEM send failed");
        return;
    }

    if (!reportedAbTelemOnline)
    {
        reportedAbTelemOnline = true;
        LOGI("ABTELEM stream online on Serial2");
    }
}

void setupTelemetrySerial()
{
    Serial2.begin(kTelemetryBaud);
    delay(100);
    telemetryStreams.addStream(Serial2);
}
} // namespace
#endif

void setup()
{
    Serial.begin(115200);
#ifndef NATIVE
#ifdef ENV_TEENSY
    setupTelemetrySerial();
#else
    Serial2.begin(115200);
#endif
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
        // .withHITL()
        .withHITLInterface(&Serial)
        ;
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
#ifdef ENV_TEENSY
    publishAbTelemetryIfDue();
#endif
}
