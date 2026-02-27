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
#include "md6.h"
#include "RuntimeHelpers.h"
#include <Sensors/HW/Mag/LIS3MDL.h>

using namespace astra;
using namespace astra_rocket;

static AstraRocketConfig config;
AstraRocket rocket(config);

static MotorDriver *g_mot = nullptr;
static VoltageSensor *g_vs = nullptr;
static AirbrakeController *g_airbrakeCtrl = nullptr;
static bool g_hitlReadySent = false;
static bool g_hitlRuntimeReady = false;
static bool g_emitCompactMain = false;
static bool g_emitCompactRadio = false;
static bool g_compactHeaderSentMain = false;
static bool g_compactHeaderSentRadio = false;
static uint32_t g_lastCompactTelemetryMs = 0;
static PrintLog g_fullMainTelemSink(Serial, true);
static ILogSink *g_hitlMainDataSinks[1] = {&g_fullMainTelemSink};

static const uint32_t STATIONARY_CAL_TIME_MS = 3000; // 3s still
static const uint32_t MAG_CAL_TIME_MS = 30000;       // 30s rotate

static constexpr double kDefaultMinAngleDeg = 0.0;
static constexpr double kDefaultMaxAngleDeg = 73.0;
static constexpr uint32_t kCompactTelemetryIntervalMs = 500; // 2 Hz

#if defined(CORE_TEENSY) && !defined(NATIVE)
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

static bool parseDoubleArg(const char *text, double &outValue)
{
    if (!text)
        return false;

    char *endPtr = nullptr;
    const double value = strtod(text, &endPtr);
    if (endPtr == text)
        return false;

    while (*endPtr == ' ' || *endPtr == '\t' || *endPtr == '\r' || *endPtr == '\n')
        endPtr++;

    if (*endPtr != '\0')
        return false;

    outValue = value;
    return true;
}

static void handleAirbrakeMessage(const char *message, const char *prefix, Stream *source)
{
    (void)prefix;

    if (!message || !source)
        return;

    char buffer[128];
    strncpy(buffer, message, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *command = strtok(buffer, " \t\r\n");
    if (!command)
    {
        source->println("AB ERR empty command");
        return;
    }

    if (strcmp(command, "TARGET_APOGEE") == 0)
    {
        if (!g_airbrakeCtrl)
        {
            source->println("AB ERR controller not ready");
            return;
        }
        const char *arg = strtok(nullptr, " \t\r\n");
        double apogeeM = 0.0;
        if (!parseDoubleArg(arg, apogeeM))
        {
            source->println("AB ERR TARGET_APOGEE requires a numeric value");
            return;
        }

        g_airbrakeCtrl->setTargetApogee(apogeeM);
        source->printf("AB OK target_apogee=%.2f\n", apogeeM);
        return;
    }

    if (strcmp(command, "ANGLE") == 0)
    {
        if (!g_mot)
        {
            source->println("AB ERR motor not ready");
            return;
        }
        const char *arg = strtok(nullptr, " \t\r\n");
        double angleDeg = 0.0;
        if (!parseDoubleArg(arg, angleDeg))
        {
            source->println("AB ERR ANGLE requires a numeric value");
            return;
        }

        if (angleDeg < kDefaultMinAngleDeg)
            angleDeg = kDefaultMinAngleDeg;
        else if (angleDeg > kDefaultMaxAngleDeg)
            angleDeg = kDefaultMaxAngleDeg;
        
        if (!g_mot->isMotorEnabled())
        {
            source->println("AB INFO enabling motor...");
            if (!g_mot->enableMotor())
            {
                source->println("AB ERR failed to enable motor");
                return;
            }
        } 

        const float targetPos = g_mot->angleToPos(static_cast<float>(angleDeg));
        g_mot->setPos(targetPos);
        source->printf("AB OK angle=%.2f pos=%.4f\n", angleDeg, targetPos);
        return;
    }

    source->println("AB ERR unknown command (use TARGET_APOGEE or ANGLE)");
}

static void handleHitlMessage(const char *message, const char *prefix, Stream *source)
{
    (void)prefix;
    if (!message || !source)
        return;

    char buffer[64];
    strncpy(buffer, message, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *command = strtok(buffer, " \t\r\n");
    if (!command)
    {
        source->println("HITL WAIT");
        return;
    }

    if ((strcmp(command, "READY?") == 0) || (strcmp(command, "PING") == 0))
    {
        source->println(g_hitlRuntimeReady ? "HITL READY" : "HITL WAIT");
        return;
    }

    source->println("HITL WAIT");
}

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
#if defined(ENV_TEENSY) && !defined(NATIVE)
    Serial2.begin(115200);
#endif
#if defined(CORE_TEENSY) && !defined(NATIVE)
    printTeensyCrashReport();
#endif
    // Construct reporters at runtime so they are reliably registered in DataLogger.
    static MotorDriver motInst("MotorDriver");
    static VoltageSensor vsInst(A0, 787, 1000, "Bat Voltage");
    static AirbrakeController airbrakeCtrlInst(&motInst, nullptr, nullptr, "AirbrakeCtrl");
    g_mot = &motInst;
    g_vs = &vsInst;
    g_airbrakeCtrl = &airbrakeCtrlInst;

    bool usingHitlSensors = false;
#if defined(NATIVE)
    config.withHITL(true);
    usingHitlSensors = true;
    Serial.println("SITL mode enabled");
    if (!Serial.connectSITL("localhost", 5555))
    {
        Serial.println("ERROR: Failed to connect to SITL server");
    }
#else
    delay(2000);
    // Hardware build running with astra-support HITL: use HITL sensors only.
    // If real sensors are configured here, Astra will keep using them and ignore HITL injections.
    // config.withHITL(true);
    // usingHitlSensors = true;
    // Serial.println("HITL mode enabled (hardware build): using HITL sensors");

    BMI088 *imu = new BMI088();
    DPS368 *rawBaro = new DPS368();
  astra::LIS3MDL *mag = new astra::LIS3MDL();

    config.with6DoFIMU(imu)
        .withMag(mag)
        .withBaro(rawBaro)
        .withGPS(new SAM_M10Q());

    imu->setMountingOrientation(MountingOrientation::FLIP_XZ); // Adjust based on your mounting
    mag->setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    // Poll mag below its default ODR to avoid repeated identical samples tripping stuck-reading health checks.
    mag->setUpdateRate(20);
#endif

    g_emitCompactMain = !usingHitlSensors;
#if defined(NATIVE)
    g_emitCompactMain = false;
#endif
#if defined(ENV_TEENSY) && !defined(NATIVE)
    g_emitCompactRadio = true;
#else
    g_emitCompactRadio = false;
#endif

    config.withMiscSensor(g_mot).withMiscSensor(g_vs);
    config.withBaroMachLockout(true, 0.7);

    if (!rocket.init())
    {
        Serial.println("ERROR: AstraRocket initialization failed!");
        LOGE("ASTRA FAILED TO INIT");
    }

    // Configure Mahony gains only after state/filter exist.
    auto *rocketState = rocket.getRocketState();
    auto *filter = rocketState ? rocketState->getOrientationFilter() : nullptr;
#if !defined(NATIVE)
    if (filter)
    {
        filter->setKp(0.8);
        filter->setKi(0.001);
    }
#else
    (void)filter;
#endif

    runtime_helpers::runOrientationCalibration(rocket,
                                               config,
                                               usingHitlSensors,
                                               Serial,
                                               STATIONARY_CAL_TIME_MS,
                                               MAG_CAL_TIME_MS);

    if (g_mot && g_mot->isInitialized())
    {
        g_mot->zeroMotor();
    }
    else
    {
        LOGE("Motor Not Initialized");
    }

    if (g_airbrakeCtrl)
    {
        g_airbrakeCtrl->setRocketState(rocket.getRocketState());
        g_airbrakeCtrl->installBaroWrapper(config.getSensorManager());
        g_airbrakeCtrl->begin();
        g_airbrakeCtrl->setTargetApogee(1300.0);
        g_airbrakeCtrl->setBinarySearchParams(10, 10.0, 5.0);
        g_airbrakeCtrl->setAngleLimits(kDefaultMinAngleDeg, kDefaultMaxAngleDeg);
        g_airbrakeCtrl->setRocketParameters(21.0, 0.01168, 0.00987);
        g_airbrakeCtrl->setGroundAltitude(137.0);
        g_airbrakeCtrl->setTransonicLockout(true, 0.7);
        g_airbrakeCtrl->setSimulationParams(0.05, 45.0);
        g_airbrakeCtrl->enableAdaptiveCdA(true, 0.2);
        g_airbrakeCtrl->enableBaroCorrection(true, 0.052, 0.15);
        g_airbrakeCtrl->enable();
    }

    Astra *astraSys = rocket.getAstraSystem();
    if (astraSys && astraSys->getMessageRouter())
    {
        astraSys->getMessageRouter()->withListener("AB/", handleAirbrakeMessage);
        astraSys->getMessageRouter()->withListener("HITL/", handleHitlMessage);
        Serial.println("AB commands enabled: AB/TARGET_APOGEE <m>, AB/ANGLE <deg>");
    }
    else
    {
        Serial.println("AB router unavailable");
    }

    // SITL and HITL tooling expect the full DataLogger header on main serial.
    if (usingHitlSensors)
    {
        // AstraRocket config has no native/USB data sink by default.
        // Rebind DataLogger to main serial in HITL/SITL so TELEM/ full schema is available.
        DataLogger::configure(g_hitlMainDataSinks, 1);
        Astra *hitlAstra = rocket.getAstraSystem();
        if (hitlAstra)
        {
            // Prime one event-driven update so HITL reporters are populated
            // before we emit the startup header expected by sim tooling.
            hitlAstra->update(0.0);
        }
        emitFullTelemHeader(Serial);
    }
}

void loop()
{
    rocket.update();

    RocketState *state = rocket.getRocketState();
    if (!state)
    {
        return;
    }

    if (!g_hitlRuntimeReady && g_mot && g_mot->isInitialized() && g_airbrakeCtrl)
    {
        g_hitlRuntimeReady = true;
    }

    if (!g_hitlReadySent)
    {
        // Emit readiness only once the main runtime loop is actively executing.
        Serial.println("HITL READY");
        g_hitlReadySent = true;
    }

    const uint32_t nowMs = millis();
    if (nowMs - g_lastCompactTelemetryMs >= kCompactTelemetryIntervalMs)
    {
        if (g_emitCompactMain)
        {
            if (!g_compactHeaderSentMain)
            {
                runtime_helpers::emitCompactHeader(Serial);
                g_compactHeaderSentMain = true;
            }
            else
            {
                runtime_helpers::emitCompactData(Serial, state, config, g_airbrakeCtrl, g_vs, g_mot);
            }
        }
#if defined(ENV_TEENSY) && !defined(NATIVE)
        if (g_emitCompactRadio)
        {
            if (!g_compactHeaderSentRadio)
            {
                runtime_helpers::emitCompactHeader(Serial2);
                g_compactHeaderSentRadio = true;
            }
            else
            {
                runtime_helpers::emitCompactData(Serial2, state, config, g_airbrakeCtrl, g_vs, g_mot);
            }
        }
#endif
        g_lastCompactTelemetryMs = nowMs;
    }

    const FlightStage stage = state->getFlightStage();
    const bool inPadIdle = (stage == PAD_IDLE);

    // In PAD_IDLE, do nothing.
    if (inPadIdle)
    {
        return;
    }

    if (g_airbrakeCtrl)
    {
        g_airbrakeCtrl->update();
    }
}
