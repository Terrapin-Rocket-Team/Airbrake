#ifndef AIRBRAKE_STATE_H
#define AIRBRAKE_STATE_H

#include <Arduino.h>
#include <MMFS.h>
#include "BR.h"
#include "vn_100.h"

enum AirbrakeStages {
    PRELAUNCH,
    BOOST,
    COAST,
    DEPLOY,
    DROUGE,
    MAIN,
    LANDED
};

// Motor driver pins
const int brk_pin = 3; 
const int stop_pin = 4; //set to low to STOP the motor, high to let the motor move (it is just an enable pin)
const int dir_pin = 5; // set low to open the airbrake, high to close the airbrake
const int speed_pin = 2; // set to 255 for full speed, set to 0 for no speed

const int stepGranularity = 7500;

class AirbrakeState: public mmfs::State{

public:

    int buzzerPin = 0;
    uint8_t currentDirection = LOW;

    // Flight configuation parameters
    // double empty_mass = 40;      // in kg
    // double target_apogee = 3100; // in m
    // double ground_altitude = 0; // in m

    // Simulated parameters
    // int max_guesses = 10;        // # of guesses before converging on desired actuation
    // int threshold = 10;          // threshold for difference between predicted and desired apogee, (m)
    // double angle_resolution = 5; // used in rounding desired angle to nearest increment

    // Airbrake Variables
    double actuationAngle = 0;            // desired actuation angle, (degrees)
    // double estimated_apogee = 0; // in m
    // double density = 1.225; // in kg/m^3 (this is just std atm denisty at sea level for initialization)
    // int cda_number_of_measurements = 0;
    // double cda_rocket;

    AirbrakeStages stage = PRELAUNCH;

    // Construtor
    AirbrakeState(mmfs::Sensor** sensors, int numSensors, mmfs::LinearKalmanFilter *kfilter, int buzzPin)
    : mmfs::State(sensors, numSensors, kfilter) {buzzerPin = buzzPin;}

    void updateState(double newTime = -1) override;

    // Helper Functions
    void setAirbrakeStage();
    void updateMotor();

    // Motor and encoder functions
    void goToStep(int step);
    void goToDegree(int degree);
    int desiredStep = 0;
    int dir_change_time = 0;

    // Airbrake functions from last year
    // int calculateActuationAngle(double altitude, double velocity, double tilt, double loop_time);
    // double predict_apogee(double time_step,double CdA_rocket, double flap_angle,double tilt, double cur_velocity, double cur_height, double ground_alt, double empty_mass);
    // double get_density(double h);
    // void update_cda_estimate();

    double timeOfLastStage; // in seconds

private:
    double timeOfLaunch; // in seconds
    

};

#endif