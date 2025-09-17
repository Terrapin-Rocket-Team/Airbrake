#ifndef AIRBRAKE_STATE_H
#define AIRBRAKE_STATE_H

#include <Arduino.h>
#include <MMFS.h>
#include <Filters/Filter.h>
#include "BR.h"

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

// Motor driver pins
const int brk_pin = 3;
const int stop_pin = 4;  // set to low to STOP the motor, high to let the motor move (it is just an enable pin)
const int dir_pin = 5;   // set low to open the airbrake, high to close the airbrake
const int speed_pin = 2; // set to 255 for full speed, set to 0 for no speed

// Limit Switch Pin
const int LIMIT_SWITCH_PIN = 6;

const int stepGranularity = 17500;

class AirbrakeState : public mmfs::State
{

public:
    // Construtor
    AirbrakeState(mmfs::Sensor **sensors, int numSensors, Filter *kfilter);

    virtual bool init(bool useBiasCorrection = false) override;

    uint8_t currentDirection = LOW;

    // Flight configuation parameters
    double full_mass = 63.5;                      // in [kg]
    double empty_mass = 43.5;                      // in [kg]
    double mass = empty_mass;                       // current step mass in [kg]
    // double predicted_target_apogee = 9144;          // in [m] 
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
    void determineStage() override;
    void updateMotor();

    // motor Stall
    bool motorStallCondition();
    static const int encoderSame = 8; // size of the circular buffer
    int historyIndex = 0;
    int encoderHistory[encoderSame]; // Circular buffer to store the last encoderSame values, size of array is the amount of the same values

    // Motor and encoder functions
    void goToStep(int step);
    void goToDegree(int degree);
    int stepToDegree(int step);
    // Number for degree to desired step: https://docs.google.com/spreadsheets/d/1bsWIpDW322UWTvhwyznNfmbBDd-kcjSC/edit?gid=1716849137#gid=1716849137
    int degreeToStepConvertionFactor = -10471; // // Negative because negative steps is open and degree defined to 0 at closed and 90 at open (v3), v1 old number: 10537, v2 old number: 9259
    int desiredStep = 0;
    int dir_change_time = 0;
    void zeroMotor();
    int motorSpeed = 255;  // value from 0 to 255

    // Airbrake functions from last year
    int calculateActuationAngle(double altitude, double velocity, double tilt);
    double predict_apogee(double time_step, double tilt, double cur_velocity, double cur_height, int flapAngle);
    double get_density(double h);
    void update_CdA_estimate();

    double timeOfLastStage; // in seconds

private:
    double timeOfLaunch; // in seconds

protected:
    void updateVariables() override;
    void updateKF() override;

    double z_accel = 0;
    double zdot_accel = 0;

    //EKF Functions/Variables
    // mmfs::Matrix X;
    // mmfs::Matrix P; // Error covariance matrix
    // mmfs::Matrix K; // Kalman gain
    // mmfs::Matrix K_super; // Kalman gain

    // Measurement noise
    // double br_std = .5; // [m]
    // double dps310_std = .2; // [m]
    // double imu_std = .1; // [m/s^2]
    // double processNoise = 10; // [m/s^2]

    // mmfs::Matrix IB;

    // void iterate(double dt, double* measurements, bool supersonic_flag);

    // void predictState(double dt);
    // void covarianceExtrapolate(double dt);
    // void calculateKalmanGain(bool supersonic_flag);
    // void estimateState(mmfs::Matrix measurement, bool supersonic_flag);
    // void covarianceUpdate(bool supersonic_flag);

    // mmfs::Matrix f(mmfs::Matrix X);
    // mmfs::Matrix getF(double dt);
    // mmfs::Matrix h(mmfs::Matrix X);
    // mmfs::Matrix getH();
    // mmfs::Matrix getR();

    // mmfs::Matrix h_super(mmfs::Matrix X);
    // mmfs::Matrix getH_super();
    // mmfs::Matrix getR_super();

    // mmfs::Matrix getQ(double dt);
};

#endif