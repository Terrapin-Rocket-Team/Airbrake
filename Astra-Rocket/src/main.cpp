#include <Arduino.h>

#include <AstraRocket.h>
#include <Sensors/HW/GPS/SAM_M10Q.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/MS5611.h>
#include <Sensors/HW/Mag/MMC5603NJ.h>
#include <Sensors/VoltageSensor/VoltageSensor.h>
#include <RecordData/Logging/DataLogger.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>

#include <cstdlib>
#include <cstring>
#include <cmath>

#include "AirbrakeController.h"
#include "md6.h"

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
astra::MS5611 rawBaro("MS5611", &Wire, (uint8_t)0x77);

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

static void emitCompactHeader(Stream &out)
{
    out.println("CTLM/t_s,stage,bat_v,ab_bat_v,pz_m,vz_mps,az_mps2,lat_deg,lon_deg,ab_cmd_deg,ab_act_deg,q_re_w,q_re_x,q_re_y,q_re_z");
}

static void emitCompactData(Stream &out, RocketState *state)
{
    double pz = 0.0;
    double vz = 0.0;
    double az = 0.0;
    double stage = 0.0;
    double batV = 0.0;
    double abBatV = 0.0;
    double lat = 0.0;
    double lon = 0.0;
    double abCmd = 0.0;
    double abAct = 0.0;
    double qw = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;

    if (state)
    {
        stage = static_cast<int>(state->getFlightStage());
        const Vector<3> pos = state->getPosition();
        const Vector<3> vel = state->getVelocity();
        const Vector<3> acc = state->getAcceleration();
        pz = pos.z();
        vz = vel.z();
        az = acc.z();

        MahonyAHRS *orientation = state->getOrientationFilter();
        if (orientation)
        {
            const Quaternion q = orientation->getQuaternion();
            qw = q.w();
            qx = q.x();
            qy = q.y();
            qz = q.z();
        }
    }

    SensorManager *sm = config.getSensorManager();
    if (sm)
    {
        GPS *gps = sm->getGPSSource();
        if (gps && gps->isInitialized() && gps->getHasFix())
        {
            const Vector<3> gpsPos = gps->getPos();
            lat = gpsPos.x();
            lon = gpsPos.y();
        }
    }

    if (g_airbrakeCtrl)
    {
        abCmd = g_airbrakeCtrl->getCommandedDeployment();
        abAct = g_airbrakeCtrl->getCurrentDeployment();
    }
    if (g_vs)
    {
        batV = g_vs->getVoltage();
    }
    if (g_mot)
    {
        abBatV = g_mot->getBatteryVoltage();
    }

    const double tSec = millis() / 1000.0;

    out.print("CTLM/");
    out.print(tSec, 3);
    out.write(',');
    out.print(stage, 0);
    out.write(',');
    out.print(batV, 3);
    out.write(',');
    out.print(abBatV, 3);
    out.write(',');
    out.print(pz, 3);
    out.write(',');
    out.print(vz, 3);
    out.write(',');
    out.print(az, 3);
    out.write(',');
    out.print(lat, 7);
    out.write(',');
    out.print(lon, 7);
    out.write(',');
    out.print(abCmd, 3);
    out.write(',');
    out.print(abAct, 3);
    out.write(',');
    out.print(qw, 6);
    out.write(',');
    out.print(qx, 6);
    out.write(',');
    out.print(qy, 6);
    out.write(',');
    out.println(qz, 6);
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

    MMC5603NJ *mag = new MMC5603NJ("MMC5603NJ", &Wire, 48);

    config.with6DoFIMU(imu)
        .withMag(mag)
        .withBaro(&rawBaro)
        .withGPS(new SAM_M10Q());

    imu->setMountingOrientation(MountingOrientation::FLIP_XZ); // Adjust based on your mounting
    mag->setMountingOrientation(MountingOrientation::ROTATE_90_Z);
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

    auto *sm = config.getSensorManager();
    auto *accelSrc = sm ? sm->getAccelSource() : nullptr;
    auto *gyroSrc = sm ? sm->getGyroSource() : nullptr;
    auto *magSrc = sm ? sm->getMagSource() : nullptr;

    const bool canRunOrientation = (filter != nullptr && accelSrc != nullptr && gyroSrc != nullptr);
    if (!canRunOrientation)
    {
        Serial.println("# Orientation calibration skipped: missing filter/accel/gyro.");
    }
    else if (usingHitlSensors)
    {
        Serial.println("# Orientation calibration skipped in HITL sensor mode.");
    }
    else
    {
        Serial.println("# ==================================");
        Serial.println("# Mahony Calibration Starting");
        Serial.println("# Phase 1: KEEP STILL");
        Serial.println("# ==================================");

        uint32_t startMs = millis();
        uint32_t lastMs = millis();

        // -------- PHASE 1: STATIONARY --------
        while (millis() - startMs < STATIONARY_CAL_TIME_MS)
        {
            rocket.update();

            uint32_t now = millis();
            double dt = (now - lastMs) * 1e-3;
            lastMs = now;
            if (dt <= 0.0)
            {
                delay(1);
                continue;
            }

            Vector<3> accel = accelSrc->getAccel();
            Vector<3> gyro = gyroSrc->getAngVel();
            filter->update(accel, gyro, dt);
            delay(5);
        }

        Serial.println("# Phase 1 Complete");

        // If mag is unavailable or unhealthy, continue without mag.
        bool magUsable = (magSrc != nullptr);
        if (magUsable && !magSrc->isHealthy())
        {
            magUsable = false;
        }

        if (!magUsable)
        {
            Serial.println("# Mag unavailable/unhealthy. Using gyro+accel fallback.");
            Serial.println("# Skipping mag calibration phase.");
        }
        else
        {
            Serial.println("# ==================================");
            Serial.println("# Phase 2: ROTATE BOARD IN ALL AXES");
            Serial.println("# 30 seconds...");
            Serial.println("# ==================================");

            startMs = millis();
            lastMs = millis();

            // -------- PHASE 2: MAG CALIBRATION --------
            while (millis() - startMs < MAG_CAL_TIME_MS)
            {
                rocket.update();

                uint32_t now = millis();
                double dt = (now - lastMs) * 1e-3;
                lastMs = now;
                if (dt <= 0.0)
                {
                    delay(1);
                    continue;
                }

                Vector<3> accel = accelSrc->getAccel();
                Vector<3> gyro = gyroSrc->getAngVel();
                Vector<3> mag = magSrc->getMag();

                const bool finiteMag = std::isfinite(mag.x()) && std::isfinite(mag.y()) && std::isfinite(mag.z());
                const bool nonZeroMag = mag.magnitude() > 1e-6;

                if (magSrc->isHealthy() && finiteMag && nonZeroMag)
                {
                    filter->update(accel, gyro, mag, dt);
                    filter->collectMagCalibrationSample(mag);
                }
                else
                {
                    // Degrade gracefully to accel+gyro only for this cycle.
                    filter->update(accel, gyro, dt);
                }

                if ((millis() - startMs) % 1000 < 20)
                    Serial.print(".");

                delay(5);
            }

            Serial.println();
            Serial.println("# Finalizing mag calibration...");
            filter->finalizeMagCalibration();

            if (filter->isMagCalibrated())
                Serial.println("# Mag calibration SUCCESS");
            else
                Serial.println("# Mag calibration FAILED (running accel+gyro fallback)");
        }

        Serial.println("# Calibration complete.");
        Serial.println("# ==================================");
    }

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
                emitCompactHeader(Serial);
                g_compactHeaderSentMain = true;
            }
            else
            {
                emitCompactData(Serial, state);
            }
        }
#if defined(ENV_TEENSY) && !defined(NATIVE)
        if (g_emitCompactRadio)
        {
            if (!g_compactHeaderSentRadio)
            {
                emitCompactHeader(Serial2);
                g_compactHeaderSentRadio = true;
            }
            else
            {
                emitCompactData(Serial2, state);
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
