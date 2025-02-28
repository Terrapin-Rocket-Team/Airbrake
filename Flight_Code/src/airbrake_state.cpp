#include <Arduino.h>

#include "airbrake_state.h"

AirbrakeState::AirbrakeState(mmfs::Sensor** sensors, int numSensors, mmfs::LinearKalmanFilter *kfilter): mmfs::State(sensors, numSensors, kfilter) {
    insertColumn(1, mmfs::INT, &stage, "Stage");
    addColumn(mmfs::DOUBLE, &actuationAngle, "Actuation Angle (deg)");
    addColumn(mmfs::DOUBLE, &CdA_rocket, "CdA");
    addColumn(mmfs::DOUBLE, &estimated_apogee, "Est Apo (m)");

};

void AirbrakeState::determineStage(){
    int timeSinceLaunch = currentTime - timeOfLaunch;
    mmfs::IMU *imu = reinterpret_cast<mmfs::IMU *>(getSensor(mmfs::IMU_));
    mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(getSensor(mmfs::BAROMETER_));
    
    if(stage == PRELAUNCH && imu->getAccelerationGlobal().z() > 40){
        mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
        bb.aonoff(mmfs::BUZZER, 200);
        stage = BOOST;
        timeOfLaunch = currentTime;
        timeOfLastStage = currentTime;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Launch detected.");
        // mmfs::getLogger().recordLogData(mmfs::INFO_, "Printing static data.");
        // for (int i = 0; i < maxNumSensors; i++)
        // {
        //     if (sensorOK(sensors[i]))
        //     {
        //         sensors[i]->setBiasCorrectionMode(false);
        //     }
        // }
    }
    else if(stage == BOOST && imu->getAccelerationGlobal().z() < 0){
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Boost detected.");
    }
    else if(stage == COAST && (currentTime - timeOfLastStage) > 1){ 
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = DEPLOY;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Coasting detected.");
    }
    else if(stage == DEPLOY && baroVelocity <= 0 && (currentTime - timeOfLastStage) > 5){
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        char logData[100];
        snprintf(logData, 100, "Apogee detected at %.2f m.", position.z());
        mmfs::getLogger().recordLogData(mmfs::INFO_, logData);
        stage = DROUGE;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Drogue detected.");
    }
    else if(stage == DROUGE && baro->getAGLAltFt() < 1000){ 
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = MAIN;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Main detected.");
    }
    else if(stage == MAIN && ((baro->getAGLAltFt() < 100) || ((currentTime - timeOfLastStage) > 60))){
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
    else if((stage == PRELAUNCH || stage == BOOST) && baro->getAGLAltFt() > 250){
        mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Launch detected. Using Backup Condition.");
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Printing static data.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
}

void AirbrakeState::goToStep(int step) {
    // Negative steps is opening the airbrake and positive is closing.
    desiredStep = step;
}

void AirbrakeState::goToDegree(int degree) {
    // Check to make sure degree is within [0, 70]
    if(degree > 70) {
        mmfs::getLogger().recordLogData(mmfs::ERROR_, "goToDegree takes angles in degrees from 0 (closed) to 70 (open). Angle greater than 70 passed. Setting degree to 70");
        degree = 70;
    } else if(degree < 0){
        mmfs::getLogger().recordLogData(mmfs::ERROR_, "goToDegree takes angles in degrees from 0 (closed) to 70 (open). Angle less than 0 passed. Setting degree to 0");
        degree = 0;
    }
    // Number for degree to desired step: https://docs.google.com/spreadsheets/d/1bsWIpDW322UWTvhwyznNfmbBDd-kcjSC/edit?gid=1716849137#gid=1716849137
    //desiredStep = -degree * 10537; // <- old number (v1)
    desiredStep = -degree * 9259; // Negative because negative steps is open and degree defined to 0 at closed and 90 at open (v2)
}

bool AirbrakeState:: motorStallCondition() {
    bool stall = true;
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS*>(getSensor(mmfs::ENCODER_));

    int currentEncoderValue = enc->getSteps();
    for (int i = 0; i < encoderSame; i++) {
        if (encoderHistory[i] != currentEncoderValue) {
            stall = false;
            break;
        }
    }

    // Update history buffer
    encoderHistory[historyIndex] = currentEncoderValue;
    historyIndex = (historyIndex + 1) % encoderSame; // Circular buffer

    return stall;
}

void AirbrakeState::updateMotor() {
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS*>(getSensor(mmfs::ENCODER_));

    // Set direction
    int step_diff = desiredStep - enc->getSteps();

    // if(motorStallCondition()){ // TODO need something that will only stall if it has been trying for awhile
    //     digitalWrite(stop_pin, HIGH);
    //     analogWrite(speed_pin, 0);
    // }
    if(step_diff > stepGranularity) {
        // Open the flaps
        if(currentDirection == HIGH) {
            // Start opening the flaps
            analogWrite(speed_pin, 128);
            digitalWrite(stop_pin, LOW);
            digitalWrite(dir_pin, HIGH);
        }
        else {
            // Switch direction first
            analogWrite(speed_pin, 0);
            digitalWrite(stop_pin, HIGH);

            if(dir_change_time == 0) {
                dir_change_time = millis();
            }
            else if(millis() - dir_change_time >= 500) {
                dir_change_time = 0;
                currentDirection = HIGH;
            }
        }
    }
    else if(step_diff < -stepGranularity) {
        // Close the flaps
        if (limitSwitchState == HIGH){
            digitalWrite(stop_pin, HIGH);
            analogWrite(speed_pin, 0);
        }
        if(currentDirection == LOW) {
            // Start closing the flaps
            analogWrite(speed_pin, 128);
            digitalWrite(stop_pin, LOW);
            digitalWrite(dir_pin, LOW);
        }
        else {
            // Switch direction first
            analogWrite(speed_pin, 0);
            digitalWrite(stop_pin, HIGH);

            if(dir_change_time == 0) {
                dir_change_time = millis();
            }
            else if(millis() - dir_change_time >= 500) {
                dir_change_time = 0;
                currentDirection = LOW;
            }
        }
    }
    else {
        // If the encoder is within the margin defined by stepGranularity, turn the motor off
        digitalWrite(stop_pin, HIGH);
        analogWrite(speed_pin, 0);
    }
}

void AirbrakeState::zeroMotor() {
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS*>(getSensor(mmfs::ENCODER_));
    int encoderSame = 8; // amount of encoder values that have to be in a row at once to zero the motor
    int encoderHistory[encoderSame]; // Circular buffer to store the last encoderSame values
    int historyIndex = 0;

    // Initialize history with the current encoder value
    int currentEncoderValue = enc->getSteps();
    for (int i = 0; i < encoderSame; i++) {
        encoderHistory[i] = currentEncoderValue;
    }

    unsigned long startTime = millis();

    // Move motor up slowly until the limit switch is clicked or the encoder stops changing values (after 1 second of the loop has passed)
    while(limitSwitchState == LOW){
        enc->update();

        int currentEncoderValue = enc->getSteps();
        Serial.print("Current Encoder Steps: ");
        Serial.println(currentEncoderValue);
    
        // Check if at least 2 second has passed before checking for encoder stalling
        if (millis() - startTime >= 2000) {
            bool encoderStopped = true;
            for (int i = 0; i < encoderSame; i++) {
                if (encoderHistory[i] != currentEncoderValue) {
                    encoderStopped = false;
                    break;
                }
            }

            if (encoderStopped) {
                break; // Exit if the encoder has not changed for encoderSame consecutive readings
            }
        }

        // Update history buffer
        encoderHistory[historyIndex] = currentEncoderValue;
        historyIndex = (historyIndex + 1) % encoderSame; // Circular buffer

        limitSwitchState = (digitalRead(LIMIT_SWITCH_PIN) == LOW);
        analogWrite(speed_pin, 128);
        digitalWrite(stop_pin, LOW);
        digitalWrite(dir_pin, HIGH);
        delay(5);
    }
    analogWrite(speed_pin, 0);
    digitalWrite(stop_pin, HIGH);
    digitalWrite(dir_pin, LOW);
    enc->setInitialSteps(enc->getSteps());
}

// Kalman filter functions
void AirbrakeState::updateKF() {
    // Based on linear kalman filter more measurements:
    // https://github.com/Terrapin-Rocket-Team/Airbrake/blob/main/matlab_state_estimation/LinearKalmanFilterMoreMeasurements.m
    mmfs::GPS *gps = reinterpret_cast<mmfs::GPS *>(getSensor(mmfs::GPS_));
    mmfs::IMU *imu = reinterpret_cast<mmfs::IMU *>(getSensor(mmfs::IMU_));
    mmfs::Barometer *baro1 = reinterpret_cast<mmfs::Barometer *>(getSensor(mmfs::BAROMETER_, 1));
    mmfs::Barometer *baro2 = reinterpret_cast<mmfs::Barometer *>(getSensor(mmfs::BAROMETER_, 2));

    double *measurements = new double[filter->getMeasurementSize()];
    double *inputs = new double[filter->getInputSize()];
    double *stateVars = new double[filter->getStateSize()];

    // Measurements 3 gps, 2 baro
    measurements[0] = sensorOK(gps) ? gps->getDisplacement().x() : 0;
    measurements[1] = sensorOK(gps) ? gps->getDisplacement().y() : 0;
    measurements[2] = baro1->getAGLAltM();
    measurements[3] = baro2->getAGLAltM();

    // imu x y z
    inputs[0] = acceleration.x() = imu->getAccelerationGlobal().x();
    inputs[1] = acceleration.y() = imu->getAccelerationGlobal().y();
    inputs[2] = acceleration.z() = imu->getAccelerationGlobal().z() - 9.81;

    stateVars[0] = position.x();
    stateVars[1] = position.y();
    stateVars[2] = position.z();
    stateVars[3] = velocity.x();
    stateVars[4] = velocity.y();
    stateVars[5] = velocity.z();

    // if (!isOutlier(filter->getStateSize(), stateVars, filter->getMeasurementSize(), measurements, 200)){
    //     filter->iterate(currentTime - lastTime, stateVars, measurements, inputs);
    // }
    filter->iterate(currentTime - lastTime, stateVars, measurements, inputs);
    
    // pos x, y, z, vel x, y, z
    position.x() = stateVars[0];
    position.y() = stateVars[1];
    position.z() = stateVars[2];
    velocity.x() = stateVars[3];
    velocity.y() = stateVars[4];
    velocity.z() = stateVars[5];

    if (sensorOK(baro1))
    {
        baroVelocity = (baro1->getAGLAltM() - baroOldAltitude) / (currentTime - lastTime);
        baroOldAltitude = baro1->getAGLAltM();
    }

    delete[] stateVars;
}

bool AirbrakeState::isOutlier(int stateSize, double* stateVars, int measSize, double* measurements, double threshold) {
    // Check for valid input sizes
    if (stateSize < 3 || measSize < 4) {
        // Invalid input sizes
        return false;
    }

    // Calculate residuals for GPS Z, Barometer 1, and Barometer 2
    double baro1Resid = std::abs(measurements[2] - stateVars[2]); // Barometer 1 Residual (4th element of measurements)
    double baro2Resid = std::abs(measurements[3] - stateVars[2]); // Barometer 2 Residual (5th element of measurements)

    // Check if any of the residuals are above the threshold
    if (baro1Resid > threshold || baro2Resid > threshold) {
        return true;  // Outlier detected
    }

    return false;  // No outlier
}

// Airbrake Functions from last year
// // Calculate Actuation Angle
void AirbrakeState::calculateActuationAngle(double altitude, double velocity, double tilt, double loop_time) { 

    int i = 0;
    // initial flap guesses
    double low = 0; 
    double high = 70;
            
    while (i < max_guesses) {
 
        estimated_apogee = predict_apogee(.5, tilt, velocity, altitude); 
        double apogee_difference = estimated_apogee - target_apogee;
        
        if (abs(apogee_difference) < threshold) {
            break;
        } else if (apogee_difference > 0) {
            low = actuationAngle;
        } else if (apogee_difference < 0) {
            high = actuationAngle;
        }
        
        actuationAngle = (high + low) / 2;
        i++;
    }
    
    // sets angles in degrees
    actuationAngle = angle_resolution*round(actuationAngle/angle_resolution);
}

// Calculate apogee
double AirbrakeState::predict_apogee(double time_step, double tilt, double cur_velocity, double cur_height) {
    // Uses RK2 (two-stage Runge-Kutta or midpoint method) for integration

    double time_intergrating = 0.0;
    double x = 0.0;
    double dx = sin(tilt)*cur_velocity;
    double y = cur_height;
    double dy = cos(tilt)*cur_velocity;
    double k1x = 0.0;
    double k1y = 0.0;
    double s1x = 0.0;
    double s1y = 0.0;
    double k2x = 0.0;
    double k2y = 0.0;
    double CdA_flaps = 4*0.95*single_flap_area*sin(actuationAngle*3.141592/180);

    while (time_intergrating < sim_time_to_apogee){
        k1x = -0.5*get_density(y+ground_altitude)*(CdA_rocket+CdA_flaps)*sqrt(dx*dx+dy*dy)*dx/empty_mass;
        k1y = -0.5*get_density(y+ground_altitude)*(CdA_rocket+CdA_flaps)*sqrt(dx*dx+dy*dy)*dy/empty_mass - 9.81;
        
        s1x = dx + (time_step * k1x);
        s1y = dy + (time_step * k1y);

        k2x = -0.5*get_density(y+ground_altitude)*(CdA_rocket+CdA_flaps)*sqrt(pow(s1x,2)+pow(s1y,2))*(s1x)/empty_mass;
        k2y = -0.5*get_density(y+ground_altitude)*(CdA_rocket+CdA_flaps)*sqrt(pow(s1x,2)+pow(s1y,2))*(s1y)/empty_mass - 9.81;

        dx += time_step*(k1x+k2x)/2;
        dy += time_step*(k1y+k2y)/2;
        x += time_step*dx;
        y += time_step*dy;
        time_intergrating += time_step;

        if (dy <= 0){
        return y;
        }
    
    }
    return y; 
}

// Calculate air density
double AirbrakeState::get_density(double h){
    // Input h in ASL [m]

    // Constants
    double R=8.31446;  //universal gas constant (J/(mol·K))
    double M=.0289652; //molar mass of air (kg/mol)
    double L=0.0065;  //temperature lapse rate in the troposphere (K/m)

    double p0=101325; //ground pressure (Pa) //
    double T0=288.15; //ground temperature (K) //

    
    density = p0*M/R/T0*pow((1-L*h/T0),((9.8*M/R/L)-1));
    return density;
}

//change from global frame to body frame.
mmfs::Vector<3> AirbrakeState::globalToBodyFrame(mmfs::Vector<3> vec) {
    mmfs::Quaternion globalquat = mmfs::Quaternion(0, vec);
    mmfs::Quaternion o_conj = orientation.conjugate();
    mmfs::Quaternion b_quat = orientation * globalquat * o_conj;
    return mmfs::Vector<3>(b_quat.x(), b_quat.y(), b_quat.z());
}

// estimate CdAs
void AirbrakeState::update_CdA_estimate() {
    CdA_number_of_measurements++;
    mmfs::Vector<3> bodyVelo = globalToBodyFrame(velocity);
    mmfs::Vector<3> bodyAccel = globalToBodyFrame(acceleration);
    double CdA_rocket_this_time_step =  (2*empty_mass*abs(bodyAccel.z()))/(get_density(position.z())*bodyVelo.z()*bodyVelo.z());
    CdA_rocket = (CdA_rocket*(CdA_number_of_measurements-1) + CdA_rocket_this_time_step)/CdA_number_of_measurements ;
}