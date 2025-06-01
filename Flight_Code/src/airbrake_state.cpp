#include <Arduino.h>

#include "airbrake_state.h"

AirbrakeState::AirbrakeState(mmfs::Sensor **sensors, int numSensors, Filter *kfilter) : mmfs::State(sensors, numSensors, kfilter)
{
    addColumn(mmfs::DOUBLE, &actuationAngle, "Actuation Angle (deg)");
    addColumn(mmfs::DOUBLE, &actualAngle, "Acutal Angle (deg)");
    addColumn(mmfs::DOUBLE_HP, &CdA_rocket, "CdA");
    addColumn(mmfs::DOUBLE, &estimated_apogee, "Est Apo (m)");
    addColumn(mmfs::DOUBLE, &target_apogee, "Target Apogee (m)");
    addColumn(mmfs::DOUBLE, &machNumber, "Mach Number");
    addColumn(mmfs::DOUBLE, &tilt, "Tilt [deg]");
    addColumn(mmfs::DOUBLE, &z_accel, "Pos - Accel Integrated [m]");
    addColumn(mmfs::DOUBLE, &zdot_accel, "Velo - Accel Integrated [m/s]");
};

bool AirbrakeState::init(bool useBiasCorrection)
{
    bool initialize = State::init(useBiasCorrection);

    // sets up the circular buffer for encoder stalling.
    for (int i = 0; i < encoderSame; i++)
    {
        encoderHistory[i] = 0;
    }

    // stateVars = new double[6];

    // double* initial_state_array = new double[6]{0, 0, 0, 0, 0, 0};
    // X = mmfs::Matrix(6,1,initial_state_array);
    // P = mmfs::Matrix::ident(6) * 1;

    return initialize;
}

void AirbrakeState::determineStage()
{
    mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(getSensor("Barometer"_i));

    if (stage == PRELAUNCH && acceleration.magnitude() > 40)
    {
        mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
        bb.aonoff(mmfs::BUZZER, 200);
        stage = BOOST;
        timeOfLaunch = currentTime;
        timeOfLastStage = currentTime;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Launch detected.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
    else if (stage == BOOST && acceleration.z() < 0)
    {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Coasting detected.");
    }
    else if (stage == COAST && machNumber < .8 && (currentTime - timeOfLastStage) > 5)
    {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = DEPLOY;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Entering Deploy Stage.");
    }
    else if (stage == DEPLOY && velocity.z() <= 0 && (currentTime - timeOfLastStage) > 10)
    {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        char logData[100];
        snprintf(logData, 100, "Apogee detected at %.2f m.", position.z());
        mmfs::getLogger().recordLogData(mmfs::INFO_, logData);
        stage = DROUGE;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Drogue detected.");
    }
    else if (stage == DROUGE && baro->getAGLAltFt() < 1000)
    {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = MAIN;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Main detected.");
    }
    else if (stage == MAIN && ((baro->getAGLAltFt() < 100) || ((currentTime - timeOfLastStage) > 60)))
    {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = LANDED;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Landing detected.");
        mmfs::getLogger().setRecordMode(mmfs::GROUND);
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Dumped data after landing.");
    }
    else if (stage == LANDED && (currentTime - timeOfLastStage) > 60)
    {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        stage = PRELAUNCH;
    }
    else if ((stage == PRELAUNCH || stage == BOOST) && (baro->getAGLAltM() > 1500) && (millis() > 60000))
    {
        mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Launch detected. Using Backup Condition.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
}

void AirbrakeState::updateVariables(){
    mmfs::GPS *gps = reinterpret_cast<mmfs::GPS *>(getSensor("GPS"_i));
    mmfs::IMU *imu = reinterpret_cast<mmfs::IMU *>(getSensor("IMU"_i));
    mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(getSensor("Barometer"_i));
    orientation = sensorOK(imu) ? imu->getOrientation() : mmfs::Quaternion(1, 0, 0, 0);

    if (filter)
        updateKF();
    else
        updateWithoutKF();

    if (sensorOK(gps))
    {
        coordinates = gps->getHasFix() ? mmfs::Vector<2>(gps->getPos().x(), gps->getPos().y()) : mmfs::Vector<2>(0, 0);
        heading = gps->getHeading();
    }
    else
    {
        coordinates = mmfs::Vector<2>(0, 0);
        heading = 0;
    }
    double dt = UPDATE_INTERVAL/1000.0;
    double zdotdot_accel = imu->getAccelerationGlobal().z();
    if(stage == 0){zdotdot_accel += 9.81;}
    zdot_accel += zdotdot_accel * dt; // z velo based only on accel
    z_accel += zdot_accel * dt; // z position based only on accel
    
    double alpha_velo = 0.3; // closer to 1, the more you trust the barometer
    double alpha_pos = 0.8; // closer to 1, the more you trust the barometer
    if(velocity.z() > 250){ // roughly mach = .75
        alpha_velo = 0;
        alpha_pos = 0;
    } else if (velocity.z() <= 0.0) {
        alpha_pos = 1;
        alpha_velo = .95;
    } else {
        alpha_pos = 1 - (velocity.z() / 250);
    }
    position.z() = alpha_pos * baro->getAGLAltM() + (1-alpha_pos) * z_accel;
    velocity.z() = alpha_velo * baroVelocity + (1-alpha_velo) * zdot_accel;
}

void AirbrakeState::updateKF()
{
    mmfs::GPS *gps = reinterpret_cast<mmfs::GPS *>(getSensor("GPS"_i));
    mmfs::IMU *imu = reinterpret_cast<mmfs::IMU *>(getSensor("IMU"_i));
    mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(getSensor("Barometer"_i));

    double *measurements = new double[filter->getMeasurementSize()];
    double *inputs = new double[filter->getInputSize()];

    // gps x y barometer z
    measurements[0] = sensorOK(gps) ? gps->getDisplacement().x() : 0;
    measurements[1] = sensorOK(gps) ? gps->getDisplacement().y() : 0;
    measurements[2] = baro->getAGLAltM();

    // imu x y z
    inputs[0] = acceleration.x() = imu->getAccelerationGlobal().x();
    inputs[1] = acceleration.y() = imu->getAccelerationGlobal().y();
    inputs[2] = acceleration.z() = imu->getAccelerationGlobal().z();

    stateVars[0] = position.x();
    stateVars[1] = position.y();
    stateVars[2] = position.z();
    stateVars[3] = velocity.x();
    stateVars[4] = velocity.y();
    stateVars[5] = velocity.z();

    filter->iterate(currentTime - lastTime, stateVars, measurements, inputs);
    // pos x, y, z, vel x, y, z
    position.x() = stateVars[0];
    position.y() = stateVars[1];
    velocity.x() = stateVars[3];
    velocity.y() = stateVars[4];

    if (sensorOK(baro))
    {
        baroVelocity = (baro->getAGLAltM() - baroOldAltitude) / (currentTime - lastTime);
        baroOldAltitude = baro->getAGLAltM();
    }
}

// void AirbrakeState::updateKF(){
//     BR *blueRaven = reinterpret_cast<BR *>(getSensor("BR"_i));
//     mmfs::IMU *imu = reinterpret_cast<mmfs::IMU *>(getSensor("IMU"_i));
//     mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(getSensor("Barometer"_i));

//     // imu x y z
//     acceleration.x() = imu->getAccelerationGlobal().x();
//     acceleration.y() = imu->getAccelerationGlobal().y();
//     acceleration.z() = imu->getAccelerationGlobal().z();

//     double* measurements = new double[5] {
//         baro->getAGLAltM() < -100 ? 0 : baro->getAGLAltM(),
//         blueRaven->getAltitude() < -100 ? 0 : blueRaven->getAltitude(),
//         acceleration.x(),
//         acceleration.y(),
//         acceleration.z()
//     };

//     bool supersonic_flag = false;
//     if (baro->getPressure() > 1000 || true){
//         delete[] measurements;

//         measurements = new double[3] {
//             acceleration.x(),
//             acceleration.y(),
//             acceleration.z()
//         };

//         supersonic_flag = true;
//     }

//     if(stage < 1){mass = full_mass;}
//     else if (stage == 1){mass -= (full_mass-empty_mass)/(burn_time) * UPDATE_INTERVAL/1000;}
//     else {mass = empty_mass;}
    
//     CdA = 4 * 0.95 * single_flap_area * sin(actualAngle * 3.141592 / 180);
//     double dt = UPDATE_INTERVAL/1000.0;
//     iterate(dt, measurements, supersonic_flag);

//     // pos x, y, z, vel x, y, z
//     position.x() = X.get(0,0);
//     position.y() = X.get(1,0);
//     position.z() = X.get(2,0);
//     velocity.x() = X.get(3,0);
//     velocity.y() = X.get(4,0);
//     velocity.z() = X.get(5,0);
// }

void AirbrakeState::goToStep(int step)
{
    // Negative steps is opening the airbrake and positive is closing.
    desiredStep = step;
}

void AirbrakeState::goToDegree(int degree)
{
    // Check to make sure degree is within [0, 70]
    if (degree > 70)
    {
        mmfs::getLogger().recordLogData(mmfs::ERROR_, "goToDegree takes angles in degrees from 0 (closed) to 70 (open). Angle greater than 70 passed. Setting degree to 70");
        degree = 70;
    }
    else if (degree < 0)
    {   
        mmfs::getLogger().recordLogData(mmfs::ERROR_, "goToDegree takes angles in degrees from 0 (closed) to 70 (open). Angle less than 0 passed. Setting degree to 0");
        degree = 0;
    }
    desiredStep = degree * degreeToStepConvertionFactor; // Negative because negative steps is open and degree defined to 0 at closed and 90 at open (v2)
}

int AirbrakeState::stepToDegree(int step)
{
    return step / degreeToStepConvertionFactor;
}

// returns if the motor is stalling. Does not affect the motor control.
bool AirbrakeState::motorStallCondition()
{
    bool stall = true;
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS *>(getSensor("Encoder"_i));

    int currentEncoderValue = enc->getSteps();
    for (int i = 0; i < encoderSame; i++)
    {
        if (encoderHistory[i] != currentEncoderValue)
        {
            stall = false;
            break;
        }
    }

    // Update history buffer
    encoderHistory[historyIndex] = currentEncoderValue;
    historyIndex = (historyIndex + 1) % encoderSame; // Circular buffer

    return stall;
}

void AirbrakeState::updateMotor()
{
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS *>(getSensor("Encoder"_i));

    // Set direction
    int step_diff = desiredStep - enc->getSteps();

    // if(motorStallCondition()){ // TODO need something that will only stall if it has been trying for awhile
    //     digitalWrite(stop_pin, HIGH);
    //     analogWrite(speed_pin, 0);
    // }
    if (step_diff > stepGranularity)
    {
        // Close the flaps
        if (digitalRead(LIMIT_SWITCH_PIN) == HIGH)
        {
            // Stop closing the flaps if the limit switch is activated
            digitalWrite(stop_pin, HIGH);
            analogWrite(speed_pin, 0);
            return;
        }
        if (currentDirection == HIGH)
        {
            // Start closing the flaps
            analogWrite(speed_pin, motorSpeed);
            digitalWrite(stop_pin, LOW);
            digitalWrite(dir_pin, HIGH);
        }
        else
        {
            // Switch direction first
            analogWrite(speed_pin, 0);
            digitalWrite(stop_pin, HIGH);

            if (dir_change_time == 0)
            {
                dir_change_time = millis();
            }
            else if (millis() - dir_change_time >= 500)
            {
                dir_change_time = 0;
                currentDirection = HIGH;
            }
        }
    }
    else if (step_diff < -stepGranularity)
    {
        // Open the flaps
        if (currentDirection == LOW)
        {
            // Start opening the flaps
            analogWrite(speed_pin, motorSpeed);
            digitalWrite(stop_pin, LOW);
            digitalWrite(dir_pin, LOW);
        }
        else
        {
            // Switch direction first
            analogWrite(speed_pin, 0);
            digitalWrite(stop_pin, HIGH);

            if (dir_change_time == 0)
            {
                dir_change_time = millis();
            }
            else if (millis() - dir_change_time >= 500)
            {
                dir_change_time = 0;
                currentDirection = LOW;
            }
        }
    }
    else
    {
        // If the encoder is within the margin defined by stepGranularity, turn the motor off
        digitalWrite(stop_pin, HIGH);
        analogWrite(speed_pin, 0);
    }
}

void AirbrakeState::zeroMotor()
{
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS *>(getSensor("Encoder"_i));

    unsigned long startTime = millis();

    // Move motor up slowly until the limit switch is clicked or the encoder stops changing values (after 1 second of the loop has passed)
    while (1)
    {
        enc->update();
        // Check if at least 2 second has passed before checking for encoder stalling
        if (millis() - startTime >= 2000)
        {
            // if (motorStallCondition())
            // {
            //     break; // Exit if the encoder has read repetitive numbers
            // }
        }
        if (digitalRead(LIMIT_SWITCH_PIN) == HIGH)
        {
            break; // Exit if the limit switch is hit
        }
        analogWrite(speed_pin, int(motorSpeed/2));
        digitalWrite(stop_pin, LOW);
        digitalWrite(dir_pin, HIGH);
        delay(5);
    }
    analogWrite(speed_pin, 0);
    digitalWrite(stop_pin, HIGH);
    digitalWrite(dir_pin, LOW);
    enc->setInitialSteps(enc->getSteps());
}

// Airbrake Functions from last year
// // Calculate Actuation Angle
int AirbrakeState::calculateActuationAngle(double altitude, double velocity, double tilt)
{

    int i = 0;
    // initial flap guesses
    double low = 0;
    double high = 70;
    actuationAngle = (low + high) / 2; // initalize to the midpoint for the binary search

    while (i < max_guesses)
    {

        estimated_apogee = predict_apogee(.05, tilt, velocity, altitude, actuationAngle);
        double apogee_difference = estimated_apogee - target_apogee;

        if (abs(apogee_difference) < threshold)
        {
            break;
        }
        else if (apogee_difference > 0)
        {
            low = actuationAngle;
        }
        else if (apogee_difference < 0)
        {
            high = actuationAngle;
        }

        actuationAngle = (high + low) / 2.0;
        i++;
    }

    // sets angles in degrees
    actuationAngle = angle_resolution * round(actuationAngle / angle_resolution);
    return static_cast<int>(actuationAngle);
}

// Calculate apogee
double AirbrakeState::predict_apogee(double time_step, double tilt, double cur_velocity, double cur_height, int flapAngle)
{
    // Uses RK2 (two-stage Runge-Kutta or midpoint method) for integration
    double time_integrating = 0.0;
    double x = 0.0;
    double dx = sin(tilt) * cur_velocity;
    double y = cur_height;
    double dy = cos(tilt) * cur_velocity;
    double k1x = 0.0;
    double k1y = 0.0;
    double s1x = 0.0;
    double s1y = 0.0;
    double k2x = 0.0;
    double k2y = 0.0;

    // int flapAngle = stepToDegree(desiredStep); // Used for only software testing
    double CdA_flaps = 4 * 0.95 * single_flap_area * sin(flapAngle * 3.141592 / 180);

    while (time_integrating < sim_time_to_apogee)
    {
        double rho = get_density(y + ground_altitude);
        k1x = -0.5 * rho * (CdA_rocket + CdA_flaps) * sqrt(dx * dx + dy * dy) * dx / empty_mass; // TODO:
        k1y = -0.5 * rho * (CdA_rocket + CdA_flaps) * sqrt(dx * dx + dy * dy) * dy / empty_mass - 9.81;

        s1x = dx + (time_step * k1x);
        s1y = dy + (time_step * k1y);

        double y_mid = y + time_step * dy / 2;
        double rho_mid = get_density(y_mid + ground_altitude);

        k2x = -0.5 * rho_mid * (CdA_rocket + CdA_flaps) * sqrt(s1x * s1x + s1y * s1y) * (s1x) / empty_mass;
        k2y = -0.5 * rho_mid * (CdA_rocket + CdA_flaps) * sqrt(s1x * s1x + s1y * s1y) * (s1y) / empty_mass - 9.81;

        dx += time_step * (k1x + k2x) / 2;
        dy += time_step * (k1y + k2y) / 2;
        x += time_step * dx;
        y += time_step * dy;
        time_integrating += time_step;

        if (dy <= 0)
        {
            return y;
        }
    }
    return y;
}

// Calculate air density
double AirbrakeState::get_density(double h)
{
    // Input h in ASL [m]

    // Constants
    double R = 8.31446;  // universal gas constant (J/(mol·K))
    double M = .0289652; // molar mass of air (kg/mol)
    double L = 0.0065;   // temperature lapse rate in the troposphere (K/m)

    double p0 = 101325; // ground pressure (Pa) //
    double T0 = 288.15; // ground temperature (K) //

    density = p0 * M / (R * T0) * pow((1 - L * h / T0), ((9.8 * M / (R * L)) - 1));
    return density;
}

// estimate CdAs
void AirbrakeState::update_CdA_estimate()
{
    double alpha = 0.2; // Smoothing factor: closer to 1 = faster response, closer to 0 = slower response

    double bodyVelo = velocity.magnitude();
    mmfs::Vector<3> dragAccel = {acceleration.x(), acceleration.y(), acceleration.z()+9.81};
    double CdA_rocket_this_time_step = (2 * empty_mass * abs(dragAccel.magnitude())) / (get_density(position.z()) * bodyVelo * bodyVelo);

    CdA_rocket = (1 - alpha) * CdA_rocket + alpha * CdA_rocket_this_time_step;

    // Top bound is higher for a few reasons
    // 1. If anything we want to overestimate CdA. Actuating when we should be closed is a worse failure mode and not actuating when we should (we can always try and make it up later)
    // 2. The predicted CdA is for when the rocket is < mach ~.8. When the rocket is going max speed (mach ~1.8) it has a much higher CdA
    if (CdA_rocket > 2 * predicted_CdA_rocket || CdA_rocket < .8 * predicted_CdA_rocket)
    {
        CdA_rocket = predicted_CdA_rocket;
    }
}


// Airbrake EKF Functions
// void AirbrakeState::iterate(double dt, double* measurements, bool supersonic_flag) {
//     // Convert arrays to matrices
//     mmfs::Matrix measurementMatrix;
//     if (supersonic_flag) {
//         measurementMatrix = mmfs::Matrix(3, 1, measurements);
//     } else {
//         measurementMatrix = mmfs::Matrix(5, 1, measurements);
//     }

//     // Kalman Filter steps
//     predictState(dt);
//     covarianceExtrapolate(dt);
//     calculateKalmanGain(supersonic_flag);
//     estimateState(measurementMatrix, supersonic_flag);
//     covarianceUpdate(supersonic_flag);
// }

// void AirbrakeState::predictState(double dt) {
//     X = X + f(X)*dt;
// }

// void AirbrakeState::covarianceExtrapolate(double dt) {
//     P = getF(dt) * P * getF(dt).transpose() + getQ(dt);
// }

// void AirbrakeState::calculateKalmanGain(bool supersonic_flag) {
//     if (supersonic_flag){
//         K_super = P * getH_super().transpose() * (getH_super() * P * getH_super().transpose() + getR_super()).inverse();
//     } else {
//         K = P * getH().transpose() * (getH() * P * getH().transpose() + getR()).inverse();
//     }
// }

// void AirbrakeState::estimateState(mmfs::Matrix measurement, bool supersonic_flag) {
//     if (supersonic_flag){
//         X = X + K_super * (measurement - h_super(X));
//     } else {
//         X = X + K * (measurement - h(X));
//     }
// }

// void AirbrakeState::covarianceUpdate(bool supersonic_flag) {
//     int n = X.getRows();
//     if (supersonic_flag){
//         P = (mmfs::Matrix::ident(n) - K_super * getH_super()) * P * (mmfs::Matrix::ident(n) - K_super * getH_super()).transpose() + K_super * getR_super() * K_super.transpose();
//     } else {
//         P = (mmfs::Matrix::ident(n) - K * getH()) * P * (mmfs::Matrix::ident(n) - K * getH()).transpose() + K * getR() * K.transpose();
//     }
// }

// mmfs::Matrix AirbrakeState::f(mmfs::Matrix X) {
//     // Unpack the state vector
//     double z  = X.get(2, 0);
//     double vx = X.get(3, 0);
//     double vy = X.get(4, 0);
//     double vz = X.get(5, 0);

//     // Constants and system parameters
//     double FD = .5 * get_density(z) * CdA_rocket * sqrt(vx*vx + vy*vy + vz*vz);

//     // Direction Cosine Matrix from body to inertial frame
//     // Assumes you have IB defined and up to date
//     double IB_33 = IB.get(2, 2);
//     double IB_11 = IB.get(0, 0);

//     double* data = new double[6]{
//         vx,
//         vy,
//         vz,
//         (-FD / mass) * std::sin(IB_33) * std::cos(IB_11),
//         (-FD / mass) * std::sin(IB_33) * std::sin(IB_11),
//         (-FD / mass) * std::cos(IB_33) - g
//     };
//     return mmfs::Matrix(6, 1, data);
// }

// mmfs::Matrix AirbrakeState::getF(double dt) {
//     // State-dependent terms (assumes last X used is valid for linearization)
//     double vx = X.get(3, 0);
//     double vy = X.get(4, 0);
//     double vz = X.get(5, 0);

//     // Orientation components (direction cosines)
//     double IB_33 = IB.get(2, 2);
//     double IB_11 = IB.get(0, 0);

//     // Compute D
//     double D = -get_density(X.get(2, 0)) * CdA_rocket / mass;

//     // Precompute sin/cos
//     double sin33 = std::sin(IB_33);
//     double cos33 = std::cos(IB_33);
//     double sin11 = std::sin(IB_11);
//     double cos11 = std::cos(IB_11);

//     // Fill Jacobian A (6x6)
//     double *A = new double[36]{
//         0, 0, 0, 1, 0, 0,
//         0, 0, 0, 0, 1, 0,
//         0, 0, 0, 0, 0, 1,

//         0, 0, 0, vx * D * sin33 * cos11, vy * D * sin33 * cos11, vz * D * sin33 * cos11,
//         0, 0, 0, vx * D * sin33 * sin11, vy * D * sin33 * sin11, vz * D * sin33 * sin11,
//         0, 0, 0, vx * D * cos33,         vy * D * cos33,         vz * D * cos33
//     };

//     // Now compute F ≈ I + A*dt + (A*dt)^2/2! + (A*dt)^3/3!
//     mmfs::Matrix A_mat(6, 6, A);
//     mmfs::Matrix I = mmfs::Matrix::ident(6);

//     mmfs::Matrix At = A_mat * dt;
//     mmfs::Matrix At2 = At * At;
//     mmfs::Matrix At3 = At2 * At;

//     mmfs::Matrix F = I + At + At2 * (0.5) + At3 * (1.0 / 6.0); // Up to 3rd-order Taylor

//     return F;
// }

// mmfs::Matrix AirbrakeState::h(mmfs::Matrix X) {
//     // Extract state variables
//     double z = X.get(2, 0);
//     double vx = X.get(3, 0);
//     double vy = X.get(4, 0);
//     double vz = X.get(5, 0);

//     // Orientation terms
//     double IB_33 = IB.get(2, 2);
//     double IB_11 = IB.get(0, 0);

//     // Compute drag force
//     double F_D_R = 0.5 * get_density(z) * CdA_rocket * sqrt(vx*vx + vy*vy + vz*vz);

//     // Compute accelerometer model components
//     double sin33 = std::sin(IB_33);
//     double cos33 = std::cos(IB_33);
//     double sin11 = std::sin(IB_11);
//     double cos11 = std::cos(IB_11);

//     double ax = -F_D_R / mass * sin33 * cos11;
//     double ay = -F_D_R / mass * sin33 * sin11;
//     double az = -F_D_R / mass * cos33;

//     // Assemble measurement vector Y (5x1)
//     double* data = new double[5]{
//         z,     // z_baro1
//         z,     // z_br
//         ax,    // x acceleration (IMU)
//         ay,    // y acceleration (IMU)
//         az - g // z acceleration (IMU minus gravity)
//     };

//     return mmfs::Matrix(5, 1, data);
// }

// mmfs::Matrix AirbrakeState::getH() {
//     // Extract velocities
//     double vx = X.get(3, 0);
//     double vy = X.get(4, 0);
//     double vz = X.get(5, 0);

//     // Orientation terms
//     double IB_33 = IB.get(2, 2);
//     double IB_11 = IB.get(0, 0);
//     double sin33 = std::sin(IB_33);
//     double cos33 = std::cos(IB_33);
//     double sin11 = std::sin(IB_11);
//     double cos11 = std::cos(IB_11);

//     // System parameters
//     double D = -get_density(X.get(2, 0)) * CdA_rocket / mass;

//     // Allocate and fill matrix
//     double* data = new double[5 * 6]{
//         0, 0, 1, 0, 0, 0,
//         0, 0, 1, 0, 0, 0,
//         0, 0, 0, vx * D * sin33 * cos11, vy * D * sin33 * cos11, vz * D * sin33 * cos11,
//         0, 0, 0, vx * D * sin33 * sin11, vy * D * sin33 * sin11, vz * D * sin33 * sin11,
//         0, 0, 0, vx * D * cos33, vy * D * cos33, vz * D * cos33,
//     };

//     return mmfs::Matrix(5, 6, data);
// }

// mmfs::Matrix AirbrakeState::getR() {
//     double *data = new double[25]{
//         dps310_std, 0, 0, 0, 0,
//         0, br_std, 0, 0, 0,
//         0, 0, imu_std, 0, 0,
//         0, 0, 0, imu_std, 0,
//         0, 0, 0, 0, imu_std,
//     };
//     return mmfs::Matrix(5, 5, data);
// }

// mmfs::Matrix AirbrakeState::h_super(mmfs::Matrix X) {
//     // Extract state variables
//     double z = X.get(2, 0);
//     double vx = X.get(3, 0);
//     double vy = X.get(4, 0);
//     double vz = X.get(5, 0);

//     // Orientation terms
//     double IB_33 = IB.get(2, 2);
//     double IB_11 = IB.get(0, 0);

//     // Compute drag force
//     double F_D_R = 0.5 * get_density(z) * CdA_rocket * sqrt(vx*vx + vy*vy + vz*vz);

//     // Compute accelerometer model components
//     double sin33 = std::sin(IB_33);
//     double cos33 = std::cos(IB_33);
//     double sin11 = std::sin(IB_11);
//     double cos11 = std::cos(IB_11);

//     double ax = -F_D_R / mass * sin33 * cos11;
//     double ay = -F_D_R / mass * sin33 * sin11;
//     double az = -F_D_R / mass * cos33;

//     // Assemble measurement vector Y (5x1)
//     double* data = new double[3]{
//         ax,    // x acceleration (IMU)
//         ay,    // y acceleration (IMU)
//         az - g // z acceleration (IMU minus gravity)
//     };

//     return mmfs::Matrix(3, 1, data);
// }

// mmfs::Matrix AirbrakeState::getH_super() {
//     // Extract velocities
//     double vx = X.get(3, 0);
//     double vy = X.get(4, 0);
//     double vz = X.get(5, 0);

//     // Orientation terms
//     double IB_33 = IB.get(2, 2);
//     double IB_11 = IB.get(0, 0);
//     double sin33 = std::sin(IB_33);
//     double cos33 = std::cos(IB_33);
//     double sin11 = std::sin(IB_11);
//     double cos11 = std::cos(IB_11);

//     // System parameters
//     double D = -get_density(X.get(2, 0)) * CdA_rocket / mass;

//     // Allocate and fill matrix
//     double* data = new double[3 * 6]{
//         0, 0, 0, vx * D * sin33 * cos11, vy * D * sin33 * cos11, vz * D * sin33 * cos11,
//         0, 0, 0, vx * D * sin33 * sin11, vy * D * sin33 * sin11, vz * D * sin33 * sin11,
//         0, 0, 0, vx * D * cos33, vy * D * cos33, vz * D * cos33,
//     };

//     return mmfs::Matrix(3, 6, data);
// }

// mmfs::Matrix AirbrakeState::getR_super() {
//     double *data = new double[9]{
//         imu_std, 0, 0,
//         0, imu_std, 0,
//         0, 0, imu_std,
//     };
//     return mmfs::Matrix(3, 3, data);
// }


// mmfs::Matrix AirbrakeState::getQ(double dt) {
//     double *data = new double[36]{
//         std::pow(dt, 4)/4, 0, 0, std::pow(dt, 3)/2, 0, 0,
//         0, std::pow(dt, 4)/4, 0, 0, std::pow(dt, 3)/2, 0,
//         0, 0, std::pow(dt, 4)/4, 0, 0, std::pow(dt, 3)/2,
//         std::pow(dt, 3)/2, 0, 0, std::pow(dt, 2), 0, 0,
//         0, std::pow(dt, 3)/2, 0, 0, std::pow(dt, 2), 0,
//         0, 0, std::pow(dt, 3)/2, 0, 0, std::pow(dt, 2)
//     };
//     return mmfs::Matrix(6, 6, data)*processNoise*processNoise;
// }






