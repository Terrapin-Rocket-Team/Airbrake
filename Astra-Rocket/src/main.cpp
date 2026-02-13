#include <Arduino.h>

#include <AstraRocket.h>
#include <Sensors/HW/GPS/SAM_M10Q.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/MS5611.h>
#include <Sensors/HW/Mag/MMC5603NJ.h>
#include <Sensors/VoltageSensor/VoltageSensor.h>

#include <cstdlib>
#include <cstring>

#include "AirbrakeController.h"
#include "md6.h"

using namespace astra;
using namespace astra_rocket;

static AstraRocketConfig config;
AstraRocket rocket(config);

MotorDriver mot("MotorDriver");
VoltageSensor vs(A0, 787, 1000, "Bat Voltage");
astra::MS5611 rawBaro("MS5611", Wire, 0x77);
AirbrakeController airbrakeCtrl(&mot, nullptr, nullptr, "AirbrakeCtrl");

static bool g_manualAirbrakeMode = false;
static double g_manualAngleDeg = 0.0;

static constexpr double kDefaultMinAngleDeg = 0.0;
static constexpr double kDefaultMaxAngleDeg = 85.0;

static double clampAirbrakeAngle(double angleDeg)
{
    if (angleDeg < kDefaultMinAngleDeg)
        return kDefaultMinAngleDeg;
    if (angleDeg > kDefaultMaxAngleDeg)
        return kDefaultMaxAngleDeg;
    return angleDeg;
}

static bool parseDoubleArg(const char *text, double &outValue)
{
    if (!text)
        return false;

    char *endPtr = nullptr;
    const double value = strtod(text, &endPtr);
    if (endPtr == text)
        return false;

    while (*endPtr == ' ' || *endPtr == '\t')
        endPtr++;

    if (*endPtr != '\0')
        return false;

    outValue = value;
    return true;
}

static void printAirbrakeHelp(Stream *source)
{
    if (!source)
        return;

    source->println("AB/HELP");
    source->println("AB/STATUS");
    source->println("AB/TARGET_APOGEE <meters>");
    source->println("AB/ANGLE <deg>    (manual hold, 0..85)");
    source->println("AB/AUTO           (return to controller)");
    source->println("AB/ENABLE");
    source->println("AB/DISABLE");
}

static void printAirbrakeStatus(Stream *source)
{
    if (!source)
        return;

    source->printf("AB STATUS mode=%s enabled=%d target_apogee=%.2f actual_angle=%.2f predicted_apogee=%.2f\n",
                   g_manualAirbrakeMode ? "MANUAL" : "AUTO",
                   airbrakeCtrl.isEnabled() ? 1 : 0,
                   airbrakeCtrl.getTargetApogee(),
                   airbrakeCtrl.getCurrentDeployment(),
                   airbrakeCtrl.getPredictedApogee());

    if (g_manualAirbrakeMode)
    {
        source->printf("AB STATUS manual_angle=%.2f\n", g_manualAngleDeg);
    }
}

static void handleAirbrakeMessage(const char *message, const char *prefix, Stream *source)
{
    (void)prefix;

    if (!message || !source)
        return;

    char buffer[128];
    strncpy(buffer, message, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *command = strtok(buffer, " \t");
    if (!command)
    {
        source->println("AB ERR empty command");
        return;
    }

    if (strcmp(command, "HELP") == 0)
    {
        printAirbrakeHelp(source);
        return;
    }

    if (strcmp(command, "STATUS") == 0)
    {
        printAirbrakeStatus(source);
        return;
    }

    if (strcmp(command, "TARGET_APOGEE") == 0)
    {
        const char *arg = strtok(nullptr, " \t");
        double apogeeM = 0.0;
        if (!parseDoubleArg(arg, apogeeM))
        {
            source->println("AB ERR TARGET_APOGEE requires a numeric value");
            return;
        }

        airbrakeCtrl.setTargetApogee(apogeeM);
        source->printf("AB OK target_apogee=%.2f\n", apogeeM);
        return;
    }

    if (strcmp(command, "ANGLE") == 0)
    {
        const char *arg = strtok(nullptr, " \t");
        double angleDeg = 0.0;
        if (!parseDoubleArg(arg, angleDeg))
        {
            source->println("AB ERR ANGLE requires a numeric value");
            return;
        }

        g_manualAirbrakeMode = true;
        g_manualAngleDeg = clampAirbrakeAngle(angleDeg);
        source->printf("AB OK mode=MANUAL angle=%.2f\n", g_manualAngleDeg);
        return;
    }

    if (strcmp(command, "AUTO") == 0)
    {
        g_manualAirbrakeMode = false;
        source->println("AB OK mode=AUTO");
        return;
    }

    if (strcmp(command, "ENABLE") == 0)
    {
        airbrakeCtrl.enable();
        source->println("AB OK enabled=1");
        return;
    }

    if (strcmp(command, "DISABLE") == 0)
    {
        g_manualAirbrakeMode = false;
        airbrakeCtrl.disable();
        source->println("AB OK enabled=0");
        return;
    }

    source->println("AB ERR unknown command (use AB/HELP)");
}

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
    config.withBaroMachLockout(true, 0.7);

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
    airbrakeCtrl.setAngleLimits(kDefaultMinAngleDeg, kDefaultMaxAngleDeg);
    airbrakeCtrl.setRocketParameters(43.5, 0.01168, 0.00987);
    airbrakeCtrl.setGroundAltitude(884.0);
    airbrakeCtrl.setTransonicLockout(true, 0.7);
    airbrakeCtrl.setSimulationParams(0.05, 45.0);
    airbrakeCtrl.enableAdaptiveCdA(true, 0.2);
    airbrakeCtrl.enableBaroCorrection(true, 0.052, 0.15);
    airbrakeCtrl.enable();

    Astra *astraSys = rocket.getAstraSystem();
    if (astraSys && astraSys->getMessageRouter())
    {
        astraSys->getMessageRouter()->withListener("AB/", handleAirbrakeMessage);
        Serial.println("AB router commands enabled (send AB/HELP)");
    }
    else
    {
        Serial.println("AB router unavailable");
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

    const FlightStage stage = state->getFlightStage();
    const bool inPadIdle = (stage == PAD_IDLE);

    // Keep controller fully inactive on the pad so preflight/manual commands can
    // move the flaps without being overwritten.
    if (inPadIdle)
    {
        if (g_manualAirbrakeMode)
        {
            mot.setPos(mot.angleToPos(g_manualAngleDeg));
        }
        return;
    }

    // Once flight has started, control authority returns to the controller.
    if (g_manualAirbrakeMode)
    {
        g_manualAirbrakeMode = false;
    }

    airbrakeCtrl.update();
}
