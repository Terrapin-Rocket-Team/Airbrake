#ifndef AIRBRAKE_CONTROLLER_H
#define AIRBRAKE_CONTROLLER_H

#include <RecordData/DataReporter/DataReporter.h>

#include "ErrorCorrectedBaro.h"

namespace astra {
class MotorDriver;
class SensorManager;
}

namespace astra_rocket {
class RocketState;
}

class AirbrakeController : public astra::DataReporter {
public:
    AirbrakeController(astra::MotorDriver *motor,
                       astra_rocket::RocketState *state,
                       astra::ErrorCorrectedBaro *baro = nullptr,
                       const char *name = "AirbrakeCtrl");

    int begin() override;
    int update(double currentTime = -1) override;

    void setRocketState(astra_rocket::RocketState *state);
    void setMotor(astra::MotorDriver *motor);
    void setBarometer(astra::ErrorCorrectedBaro *baro);

    void enable();
    void disable();
    bool isEnabled() const { return enabled; }

    void setTargetApogee(double targetM);
    void setRocketParameters(double massKg, double cdArocket, double flapAreaM2);
    void setBinarySearchParams(int maxIter, double thresholdM, double angleResolutionDeg);
    void setAngleLimits(double minDeg, double maxDeg);
    void setSimulationParams(double timeStepS, double maxTimeS);
    void enableAdaptiveCdA(bool enable, double alpha = 0.2);
    void enableBaroCorrection(bool enable, double c = 0.052, double tau = 0.15);
    void setGroundAltitude(double altitudeM);
    void setTransonicLockout(bool enable, double machThreshold = 0.7);
    bool installBaroWrapper(astra::SensorManager *sensorManager);

    double getPredictedApogee() const { return estimatedApogee; }
    double getCommandedDeployment() const { return actuationAngle; }
    double getCurrentDeployment() const { return actualAngle; }
    double getTargetApogee() const { return targetApogee; }

private:
    int calculateActuationAngle(double altitude, double velocity, double tiltDeg);
    double predictApogee(double timeStep, double tiltDeg, double curVelocity, double curHeight, double flapAngleDeg);
    double getDensity(double altitudeASL);
    double getSpeedOfSound(double altitudeASL);
    void updateCdAEstimate();

    astra::MotorDriver *motor = nullptr;
    astra_rocket::RocketState *state = nullptr;
    astra::ErrorCorrectedBaro *baro = nullptr;
    astra::ErrorCorrectedBaro correctedBaro;

    bool enabled = true;
    bool adaptiveCdAEnabled = true;
    bool baroCorrectionEnabled = true;
    double baroCorrectionC = 0.052;
    double baroCorrectionTau = 0.15;
    double adaptiveCdAAlpha = 0.2;

    double targetApogee = 9144.0;
    double groundAltitude = 884.0;
    double simTimeStep = 0.05;
    double simTimeMax = 45.0;
    int maxGuesses = 10;
    double threshold = 10.0;
    double angleResolution = 5.0;
    double minAngle = 0.0;
    double maxAngle = 73.0;

    double rocketMass = 43.5;
    double predictedCdArocket = 0.01168;
    double cdArocket = 0.01168;
    double flapArea = 0.00987;
    double flapEfficiency = 0.95;

    double actuationAngle = 0.0;
    double actualAngle = 0.0;
    double estimatedApogee = 0.0;
    double dynamicPressure = 0.0;
    double machNumber = 0.0;
    bool transonicLockoutEnabled = true;
    double transonicLockoutMach = 0.7;
    double transonicLockoutActive = 0.0;
};

#endif // AIRBRAKE_CONTROLLER_H
