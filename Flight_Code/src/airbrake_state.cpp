#include <Arduino.h>

#include "airbrake_state.h"
// #include "vn_100.h"

AirbrakeState::AirbrakeState(mmfs::Sensor **sensors, int numSensors, mmfs::LinearKalmanFilter *kfilter) : mmfs::State(sensors, numSensors, kfilter)
{
    insertColumn(1, mmfs::INT, &stage, "Stage");
    addColumn(mmfs::DOUBLE, &actuationAngle, "Actuation Angle (deg)");
    addColumn(mmfs::DOUBLE, &actualAngle, "Acutal Angle (deg)");
    addColumn(mmfs::DOUBLE_HP, &CdA_rocket, "CdA");
    addColumn(mmfs::DOUBLE, &estimated_apogee, "Est Apo (m)");
    addColumn(mmfs::DOUBLE, &target_apogee, "Target Apogee (m)");
    addColumn(mmfs::DOUBLE, &machNumber, "Mach Number");
    addColumn(mmfs::DOUBLE, &tilt, "Tilt [deg]");
};

bool AirbrakeState::init(bool useBiasCorrection)
{
    bool initialize = State::init(useBiasCorrection);

    // sets up the circular buffer for encoder stalling.
    for (int i = 0; i < encoderSame; i++)
    {
        encoderHistory[i] = 0;
    }
    return initialize;
}

void AirbrakeState::determineStage()
{
    mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(getSensor(mmfs::BAROMETER_));

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
    else if (stage == COAST && machNumber < .8)
    {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = DEPLOY;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Entering Deploy Stage.");
    }
    else if (stage == DEPLOY && velocity.z() <= 0 && (currentTime - timeOfLastStage) > 5)
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
        errorHandler.addError(mmfs::GENERIC_ERROR, "goToDegree takes angles in degrees from 0 (closed) to 70 (open). Angle greater than 70 passed. Setting degree to 70");
        degree = 70;
    }
    else if (degree < 0)
    {
        errorHandler.addError(mmfs::GENERIC_ERROR, "goToDegree takes angles in degrees from 0 (closed) to 70 (open). Angle less than 0 passed. Setting degree to 0");
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
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS *>(getSensor(mmfs::ENCODER_));

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
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS *>(getSensor(mmfs::ENCODER_));

    actualAngle = stepToDegree(enc->getSteps());

    // Set direction
    int step_diff = desiredStep - enc->getSteps();

    // if(motorStallCondition()){ // TODO need something that will only stall if it has been trying for awhile
    //     digitalWrite(stop_pin, HIGH);
    //     analogWrite(speed_pin, 0);
    // }
    if (step_diff > stepGranularity)
    {
        // Close the flaps
        if (limitSwitchState == HIGH)
        {
            // Stop closing the flaps if the limit switch is activated
            digitalWrite(stop_pin, HIGH);
            analogWrite(speed_pin, 0);
            return;
        }
        if (currentDirection == HIGH)
        {
            // Start closing the flaps
            analogWrite(speed_pin, 128);
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
            analogWrite(speed_pin, 128);
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
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS *>(getSensor(mmfs::ENCODER_));

    unsigned long startTime = millis();

    // Move motor up slowly until the limit switch is clicked or the encoder stops changing values (after 1 second of the loop has passed)
    while (1)
    {
        enc->update();
        // Serial.print("Encoder Steps: ");
        // Serial.println(enc->getSteps());
        // Check if at least 2 second has passed before checking for encoder stalling
        if (millis() - startTime >= 2000)
        {
            if (motorStallCondition())
            {
                break; // Exit if the encoder has read repetitive numbers
            }
        }
        if (digitalRead(LIMIT_SWITCH_PIN) == LOW)
        {
            break; // Exit if the limit switch is hit
        }
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

        estimated_apogee = predict_apogee(.5, tilt, velocity, altitude);
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
double AirbrakeState::predict_apogee(double time_step, double tilt, double cur_velocity, double cur_height)
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
    auto *enc = reinterpret_cast<mmfs::Encoder_MMFS *>(getSensor(mmfs::ENCODER_));
    int flapAngle = stepToDegree(enc->getSteps()); // Used for encoder in the loop testing
    double CdA_flaps = 4 * 0.95 * single_flap_area * sin(flapAngle * 3.141592 / 180);

    while (time_integrating < sim_time_to_apogee)
    {
        k1x = -0.5 * get_density(y + ground_altitude) * (CdA_rocket + CdA_flaps) * sqrt(dx * dx + dy * dy) * dx / empty_mass; // TODO:
        k1y = -0.5 * get_density(y + ground_altitude) * (CdA_rocket + CdA_flaps) * sqrt(dx * dx + dy * dy) * dy / empty_mass - 9.81;

        s1x = dx + (time_step * k1x);
        s1y = dy + (time_step * k1y);

        k2x = -0.5 * get_density(y + ground_altitude) * (CdA_rocket + CdA_flaps) * sqrt(pow(s1x, 2) + pow(s1y, 2)) * (s1x) / empty_mass;
        k2y = -0.5 * get_density(y + ground_altitude) * (CdA_rocket + CdA_flaps) * sqrt(pow(s1x, 2) + pow(s1y, 2)) * (s1y) / empty_mass - 9.81;

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
void AirbrakeState::update_CdA_estimate(double bodyAccelZ)
{
    CdA_number_of_measurements++;
    double bodyVelo = velocity.magnitude();
#ifdef TEST_WITH_SERIAL
    bodyAccelZ = acceleration.magnitude();
#endif
    double CdA_rocket_this_time_step = (2 * empty_mass * abs(bodyAccelZ)) / (get_density(position.z()) * bodyVelo * bodyVelo);
    CdA_rocket = (CdA_rocket * (CdA_number_of_measurements - 1) + CdA_rocket_this_time_step) / CdA_number_of_measurements;

    if (CdA_rocket > 1.3 * predicted_CdA_rocket || CdA_rocket < .7 * predicted_CdA_rocket)
    {
        CdA_rocket = predicted_CdA_rocket;
    }
}