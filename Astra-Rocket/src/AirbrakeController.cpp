#include "AirbrakeController.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <AstraRocket.h>
#include <Math/Vector.h>
#include <Sensors/SensorManager/SensorManager.h>

#include "ErrorCorrectedBaro.h"
#include "MotorDriver/MotorDriver.h"

using astra::Vector;

AirbrakeController::AirbrakeController(MotorDriver *motorIn,
                                       astra_rocket::RocketState *stateIn,
                                       astra::ErrorCorrectedBaro *baroIn,
                                       const char *name)
    : DataReporter(name),
      motor(motorIn),
      state(stateIn),
      baro(baroIn)
{
    setAutoUpdate(false);
    addColumn("%0.3f", &actuationAngle, "Actuation Angle (deg)");
    addColumn("%0.3f", &actualAngle, "Actual Angle (deg)");
    addColumn("%0.3f", &estimatedApogee, "Pred Apogee (m)");
    addColumn("%0.3f", &targetApogee, "Target Apogee (m)");
    addColumn("%0.6f", &cdArocket, "CdA Rocket");
    addColumn("%0.2f", &dynamicPressure, "Dyn Press (hPa)");
    addColumn("%0.3f", &machNumber, "Mach");
    addColumn("%0.0f", &transonicLockoutActive, "Transonic Lockout");
}

int AirbrakeController::begin()
{
    if (!motor || !state)
    {
        return -1;
    }
    return 0;
}

int AirbrakeController::update()
{
    if (!enabled || !motor || !state)
    {
        if (baro && baroCorrectionEnabled)
        {
            baro->setCorrectionInputs(0.0, 0.0);
        }
        return -1;
    }
    if (!motor->isInitialized())
    {
        if (baro && baroCorrectionEnabled)
        {
            baro->setCorrectionInputs(0.0, 0.0);
        }
        return -1;
    }
    const astra_rocket::FlightStage stage = state->getFlightStage(); 
    if(stage == astra_rocket::FlightStage::PAD_IDLE)
    {
        return 0;
    }

    const Vector<3> pos = state->getPosition();
    const Vector<3> vel = state->getVelocity();
    const double altitude = pos.z();
    const double speed = vel.magnitude();
    const double horizontalSpeed = sqrt(vel.x() * vel.x() + vel.y() * vel.y());
    const double verticalSpeed = vel.z();
    const double altitudeASL = altitude + groundAltitude;
    const double speedOfSound = getSpeedOfSound(altitudeASL);
    machNumber = (speedOfSound > 1e-6) ? (speed / speedOfSound) : 0.0;
    const bool transonicLockout = transonicLockoutEnabled && (machNumber >= transonicLockoutMach);
    transonicLockoutActive = transonicLockout ? 1.0 : 0.0;
    const bool ascending = (verticalSpeed > 0.2);
    const bool controlWindowOpen = (stage == astra_rocket::COAST) && ascending && !transonicLockout;

    if (adaptiveCdAEnabled && controlWindowOpen)
    {
        updateCdAEstimate();
    }
    else
    {
        // Keep predictor model stable before control window opens.
        cdArocket = predictedCdArocket;
    }

    if (controlWindowOpen)
    {
        actuationAngle = calculateActuationAngle(altitude, horizontalSpeed, verticalSpeed);
        if (actuationAngle < minAngle)
        {
            actuationAngle = minAngle;
        }
        else if (actuationAngle > maxAngle)
        {
            actuationAngle = maxAngle;
        }
        motor->setPos(motor->angleToPos(actuationAngle));
    }
    else
    {
        actuationAngle = minAngle;
        motor->setPos(motor->angleToPos(actuationAngle));
    }

    actualAngle = motor->posToAngle(motor->getPosition());

    if (verticalSpeed > 0.01)
    {
        // Before control is allowed, predict with locked flaps and baseline CdA.
        const double predictionFlapAngle = controlWindowOpen ? actualAngle : minAngle;
        estimatedApogee = predictApogee(simTimeStep, horizontalSpeed, verticalSpeed, altitude, predictionFlapAngle);
    }
    else
    {
        // Once vertical velocity is no longer upward, apogee has been reached.
        estimatedApogee = altitude;
    }

    if (baro && baroCorrectionEnabled)
    {
        const double rho = getDensity(altitudeASL);
        dynamicPressure = 0.5 * rho * speed * speed / 100.0; // Pa -> hPa
        baro->setCorrectionInputs(actualAngle, dynamicPressure);
    }

    return 0;
}

void AirbrakeController::setRocketState(astra_rocket::RocketState *stateIn)
{
    state = stateIn;
}

void AirbrakeController::setMotor(MotorDriver *motorIn)
{
    motor = motorIn;
}

void AirbrakeController::setBarometer(astra::ErrorCorrectedBaro *baroIn)
{
    baro = baroIn;
}

void AirbrakeController::enable()
{
    enabled = true;
}

void AirbrakeController::disable()
{
    enabled = false;
}

void AirbrakeController::setTargetApogee(double targetM)
{
    targetApogee = targetM;
}

void AirbrakeController::setRocketParameters(double massKg, double cdArocketIn, double flapAreaM2)
{
    rocketMass = massKg;
    predictedCdArocket = cdArocketIn;
    cdArocket = cdArocketIn;
    flapArea = flapAreaM2;
}

void AirbrakeController::setBinarySearchParams(int maxIter, double thresholdM, double angleResolutionDeg)
{
    maxGuesses = maxIter;
    threshold = thresholdM;
    angleResolution = angleResolutionDeg;
}

void AirbrakeController::setAngleLimits(double minDeg, double maxDeg)
{
    minAngle = minDeg;
    maxAngle = maxDeg;
}

void AirbrakeController::setSimulationParams(double timeStepS, double maxTimeS)
{
    simTimeStep = timeStepS;
    simTimeMax = maxTimeS;
}

void AirbrakeController::enableAdaptiveCdA(bool enable, double alpha)
{
    adaptiveCdAEnabled = enable;
    adaptiveCdAAlpha = alpha;
}

void AirbrakeController::enableBaroCorrection(bool enable, double c, double tau)
{
    baroCorrectionEnabled = enable;
    baroCorrectionC = c;
    baroCorrectionTau = tau;
    if (baro)
    {
        baro->setCorrectionEnabled(enable);
        baro->setCorrectionParams(c, tau);
    }
}

void AirbrakeController::setGroundAltitude(double altitudeM)
{
    groundAltitude = altitudeM;
}

void AirbrakeController::setTransonicLockout(bool enable, double machThreshold)
{
    transonicLockoutEnabled = enable;
    if (machThreshold > 0.0)
    {
        transonicLockoutMach = machThreshold;
    }
}

bool AirbrakeController::installBaroWrapper(astra::SensorManager *sensorManager)
{
    if (!sensorManager)
    {
        return false;
    }
    astra::Barometer *inner = sensorManager->getBaroSource();
    if (!inner)
    {
        return false;
    }
    if (inner == &correctedBaro)
    {
        baro = &correctedBaro;
        return true;
    }

    correctedBaro.setInnerBarometer(inner);
    correctedBaro.setCorrectionEnabled(baroCorrectionEnabled);
    correctedBaro.setCorrectionParams(baroCorrectionC, baroCorrectionTau);

    sensorManager->setBaroSource(&correctedBaro);
    baro = &correctedBaro;

    if (!baro->isInitialized())
    {
        baro->begin();
    }

    return true;
}

int AirbrakeController::calculateActuationAngle(double altitude, double horizontalVelocity, double verticalVelocity)
{
    int i = 0;
    double low = minAngle;
    double high = maxAngle;
    actuationAngle = 0.5 * (low + high);

    while (i < maxGuesses)
    {
        estimatedApogee = predictApogee(simTimeStep, horizontalVelocity, verticalVelocity, altitude, actuationAngle);
        const double diff = estimatedApogee - targetApogee;

        if (fabs(diff) < threshold)
        {
            break;
        }
        if (diff > 0)
        {
            low = actuationAngle;
        }
        else
        {
            high = actuationAngle;
        }

        actuationAngle = 0.5 * (high + low);
        i++;
    }

    actuationAngle = angleResolution * round(actuationAngle / angleResolution);
    return static_cast<int>(actuationAngle);
}

double AirbrakeController::predictApogee(double timeStep,
                                         double curHorizontalVelocity,
                                         double curVerticalVelocity,
                                         double curHeight,
                                         double flapAngleDeg)
{
    double timeIntegrating = 0.0;
    double dx = curHorizontalVelocity;
    double y = curHeight;
    double dy = curVerticalVelocity;
    double k1x = 0.0;
    double k1y = 0.0;
    double s1x = 0.0;
    double s1y = 0.0;
    double k2x = 0.0;
    double k2y = 0.0;

    const double flapAngleRad = flapAngleDeg * M_PI / 180.0;
    const double cdAflaps = 4.0 * flapEfficiency * flapArea * sin(flapAngleRad);

    while (timeIntegrating < simTimeMax)
    {
        const double rho = getDensity(y + groundAltitude);
        const double speed = sqrt(dx * dx + dy * dy);
        k1x = -0.5 * rho * (cdArocket + cdAflaps) * speed * dx / rocketMass;
        k1y = -0.5 * rho * (cdArocket + cdAflaps) * speed * dy / rocketMass - 9.81;

        s1x = dx + (timeStep * k1x);
        s1y = dy + (timeStep * k1y);

        const double yMid = y + timeStep * dy / 2.0;
        const double rhoMid = getDensity(yMid + groundAltitude);
        const double speedMid = sqrt(s1x * s1x + s1y * s1y);

        k2x = -0.5 * rhoMid * (cdArocket + cdAflaps) * speedMid * s1x / rocketMass;
        k2y = -0.5 * rhoMid * (cdArocket + cdAflaps) * speedMid * s1y / rocketMass - 9.81;

        dx += timeStep * (k1x + k2x) / 2.0;
        dy += timeStep * (k1y + k2y) / 2.0;
        y += timeStep * dy;
        timeIntegrating += timeStep;

        if (dy <= 0)
        {
            return y;
        }
    }

    return y;
}

double AirbrakeController::getDensity(double h)
{
    const double R = 8.31446;
    const double M = 0.0289652;
    const double L = 0.0065;
    const double p0 = 101325.0;
    const double T0 = 288.15;
    return p0 * M / (R * T0) * pow((1.0 - L * h / T0), ((9.8 * M / (R * L)) - 1.0));
}

double AirbrakeController::getSpeedOfSound(double h)
{
    const double T0 = 288.15;
    const double L = 0.0065;
    const double gamma = 1.4;
    const double R = 287.05;
    const double T = fmax(216.65, T0 - L * h);
    return sqrt(gamma * R * T);
}

void AirbrakeController::updateCdAEstimate()
{
    const Vector<3> vel = state->getVelocity();
    const Vector<3> acc = state->getAcceleration();
    const double speed = vel.magnitude();
    if (speed < 0.1)
    {
        return;
    }

    Vector<3> dragAccel(acc.x(), acc.y(), acc.z());
    const double rho = getDensity(state->getPosition().z() + groundAltitude);
    const double cdAestimate = (2.0 * rocketMass * dragAccel.magnitude()) / (rho * speed * speed);
    cdArocket = (1.0 - adaptiveCdAAlpha) * cdArocket + adaptiveCdAAlpha * cdAestimate;

    const double minCdA = 0.8 * predictedCdArocket;
    const double maxCdA = 2.0 * predictedCdArocket;
    if (cdArocket < minCdA)
    {
        cdArocket = minCdA;
    }
    else if (cdArocket > maxCdA)
    {
        cdArocket = maxCdA;
    }
}
