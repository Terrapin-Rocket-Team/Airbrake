#ifndef AIRBRAKE_STATE_H
#define AIRBRAKE_STATE_H

#include <Arduino.h>
#include <MMFS.h>


class AirbrakeState: public mmfs::State{

public:

    // Flight configuation parameters
    double empty_mass = 40;      // in kg
    double target_apogee = 3100; // in m

    // Simulated parameters
    int max_guesses = 10;        // # of guesses before converging on desired actuation
    int threshold = 10;          // threshold for difference between predicted and desired apogee, (m)
    double angle_resolution = 5; // used in rounding desired angle to nearest increment

    // Constants
    double R=8.31446;  //Ideal gas constant
    double M=.0289652; //molar mass of air

    // Airbrake Variables
    double guess = 0;            // desired actuation angle, (degrees)
    double estimated_apogee = 0; // in m

    // Construtor
    AirbrakeState::AirbrakeState(mmfs::Sensor** sensors, int numSensors, mmfs::KalmanInterface* kfilter, bool stateRecordsOwnData)
    : mmfs::State(sensors, numSensors, kfilter, stateRecordsOwnData) {}

    // Helper Functions
    int calculateActuationAngle(double altitude, double velocity, double targetApogee, double tilt, double CdA_rocket);
    double predict_apogee(double time_step,double CdA_rocket, double flap_angle,double tilt, double cur_velocity, double cur_height, double ground_alt, double empty_mass);
    double get_density(double h);

private:

};

#endif