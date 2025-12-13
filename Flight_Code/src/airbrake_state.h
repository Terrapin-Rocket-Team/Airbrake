#ifndef AIRBRAKE_STATE_H
#define AIRBRAKE_STATE_H

#include <Arduino.h>
#include <State/State.h>
#include <Filters/Filter.h>

enum AirbrakeStages
{
    PRELAUNCH,
    BOOST,
    COAST,
    DEPLOY,
    DROUGE,
    MAIN,
    LANDED
};

class AirbrakeState : public astra::State
{

public:
    // Construtor
    AirbrakeState(astra::Sensor **sensors, int numSensors, Filter *kfilter);

    uint8_t currentDirection = LOW;

    // Flight configuation parameters
    double full_mass = 63.5;                      // in [kg]
    double empty_mass = 43.5;                      // in [kg]
    double mass = empty_mass;                       // current step mass in [kg]
    double target_apogee = 9144;                    // in [m] (30000 ft)
    double ground_altitude = 884;                  // ASL in [m]
    double sim_time_to_apogee = 45;                 // in [s]
    double burn_time  = 4.83;                       // estimated burn time of motor in [s]
    double g = 9.81;                                // in [m/s^2]

    // Simulated parameters
    int max_guesses = 10;        // number of guesses before converging on desired actuation
    int threshold = 10;          // threshold for difference between predicted and desired apogee, [m]
    double angle_resolution = 5; // used in rounding desired angle to nearest increment

    // Airbrake Variables
    double actuationAngle = 0;   // desired actuation angle, (degrees)
    double actualAngle = 0;      // actual actuation angle, (degrees)
    double estimated_apogee = 0; // in [m]
    double density = 1.225;      // in [kg/m^3] (this is just std atm denisty at sea level for initialization)
    double predicted_CdA_rocket = .62 * 0.01885; // CDr*Area (6.1in): Will get updated during flight but initial set based on: https://drive.google.com/drive/u/0/folders/150lm54Gioq1RoHnZDieAeiPLmdDmVhk5
    double CdA_rocket = predicted_CdA_rocket;
    double CdA; // CdA flaps + CdA rocket
    double single_flap_area = 0.00987; // measured
    double machNumber = 0;
    double tilt = 4; // [deg]

    // Barometric error correction (based on https://drive.google.com/drive/u/0/folders/150lm54Gioq1RoHnZDieAeiPLmdDmVhk5)
    double altitudeDelta = 0; // [m]
    const double c = 0.052; // [~]
    const double tau = .15; // [~]
    const double baroAlpha = .1/(.1 + tau); // [~]

    AirbrakeStages stage = PRELAUNCH;

    // Helper Functions
    void determineStage();

    // Airbrake flap angle calculation
    int calculateActuationAngle(double altitude, double velocity, double tilt);
    double predict_apogee(double time_step, double tilt, double cur_velocity, double cur_height, int flapAngle);
    double get_density(double h);
    void update_CdA_estimate();

    double timeOfLastStage; // in seconds

private:
    double timeOfLaunch; // in seconds

protected:
    void updateVariables();

    double z_accel = 0;
    double zdot_accel = 0;

};

#endif