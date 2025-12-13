#include <Arduino.h>

#include "airbrake_state.h"
#include <RecordData/Logging/EventLogger.h>

AirbrakeState::AirbrakeState(astra::Sensor **sensors, int numSensors, Filter *kfilter) : astra::State(sensors, numSensors, kfilter)
{
    insertColumn(1,"%d", &stage, "Stage");
    addColumn("%0.3f", &actuationAngle, "Actuation Angle (deg)");
    addColumn("%0.3f", &actualAngle, "Acutal Angle (deg)");
    addColumn("%0.7f", &CdA_rocket, "CdA");
    addColumn("%0.3f", &estimated_apogee, "Est Apo (m)");
    addColumn("%0.3f", &target_apogee, "Target Apogee (m)");
    addColumn("%0.3f", &machNumber, "Mach Number");
    addColumn("%0.3f", &tilt, "Tilt [deg]");
    addColumn("%0.3f", &z_accel, "Pos - Accel Integrated [m]");
    addColumn("%0.3f", &zdot_accel, "Velo - Accel Integrated [m/s]");
    addColumn("%0.3f", &altitudeDelta, "Baro Correction Altitude Delta [m]");
};

void AirbrakeState::determineStage()
{
    astra::Barometer *baro = reinterpret_cast<astra::Barometer *>(getSensor("Barometer"_i));

    //boost
    if (stage == PRELAUNCH && acceleration.magnitude() > 40)
    {
        // astra::getLogger().setRecordMode(astra::FLIGHT);
        bb.aonoff(astra::BUZZER, 200);
        stage = BOOST;
        timeOfLaunch = currentTime;
        timeOfLastStage = currentTime;
        LOGI("Launch detected.");
    }
    //Coast
    else if (stage == BOOST && acceleration.z() < 0)
    {
        bb.aonoff(astra::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        LOGI("Coasting detected.");
    }
    //Deploy
    else if (stage == COAST && machNumber < .8 && (currentTime - timeOfLastStage) > 2)
    {
        bb.aonoff(astra::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = DEPLOY;
        LOGI("Entering Deploy Stage.");
    }
    //Drogue
    else if (stage == DEPLOY && velocity.z() < 0 && (currentTime - timeOfLastStage) > 5)
    {
        bb.aonoff(astra::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        char logData[100];
        snprintf(logData, 100, "Apogee detected at %.2f m.", position.z());
        LOGI(logData);
        stage = DROUGE;
        LOGI("Drogue detected.");
    }
    //Main
    else if (stage == DROUGE && baro->getASLAltFt() < 1000)
    {
        bb.aonoff(astra::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = MAIN;
        LOGI("Main detected.");
    }
    //Landed
    else if (stage == MAIN && ((baro->getASLAltFt() < 100) || ((currentTime - timeOfLastStage) > 60)))
    {
        bb.aonoff(astra::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = LANDED;
        LOGI("Landing detected.");
        //astra::getLogger().setRecordMode(astra::GROUND);
        LOGI("Dumped data after landing.");
    }
    //Reset to PRELAUNCH
    else if (stage == LANDED && (currentTime - timeOfLastStage) > 60)
    {
        bb.aonoff(astra::BUZZER, 200, 2);
        stage = PRELAUNCH;
    }
    //Backup Launch Detection
    else if ((stage == PRELAUNCH || stage == BOOST) && (baro->getASLAltM() > 2500) && (millis() > 60000))
    {
        //astra::getLogger().setRecordMode(astra::FLIGHT);
        bb.aonoff(astra::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        LOGI("Launch detected. Using Backup Condition.");
    }
}

void AirbrakeState::updateVariables(){

    astra::State::updateVariables();
    
    // Do Barometric error correction
    if (stage == DEPLOY){
        double altitudeErrorFunction;
        double q = .5 * get_density(position.z()) * velocity.magnitude() * velocity.magnitude();
        if(actualAngle > 45) {altitudeErrorFunction = c*q*sin(45 * M_PI / 180);}
        else {altitudeErrorFunction = c*q*sin(actualAngle * M_PI / 180);}
        altitudeDelta = (1 - baroAlpha)*altitudeDelta + baroAlpha*altitudeErrorFunction;
    } 

    if (stage == PRELAUNCH){return;}
    
    // Get postion and velocity from integrating acceleration
    double dt = (currentTime-lastTime)/1000.0;
    //double zdotdot_accel = imu->getAccelerationGlobal().z();
    // if(stage == 0){zdotdot_accel += 9.81;}
    // zdot_accel += zdotdot_accel * dt; // z velo based only on accel
    // z_accel += zdot_accel * dt; // z position based only on accel

    // astra::Vector<3> machVelo;
    // machVelo.x() = velocity.x(); machVelo.y() = velocity.y(); machVelo.z() = zdot_accel;
    // machNumber = machVelo.magnitude() / sqrt(1.4 * 286 * (baro->getTemp() + 273.15)); // M = V/sqrt(gamma*R*T)
    // if(machNumber > 2.5 || machNumber < 0){
    //     velocity.z() = zdot_accel;
    //     machNumber = velocity.magnitude() / sqrt(1.4 * 286 * (baro->getTemp() + 273.15)); // M = V/sqrt(gamma*R*T)
    //     position.z() = z_accel;
    // }

    // Serial.println(machNumber);
    
    // double alpha_velo = 0.3; // closer to 1, the more you trust the barometer
    // double alpha_pos = 0.8; // closer to 1, the more you trust the barometer
    // if(velocity.z() > 250){ // roughly mach = .75
    //     alpha_velo = 0;
    //     alpha_pos = 0;
    // } else if (velocity.z() <= 0.0) {
    //     alpha_pos = 1;
    //     alpha_velo = .95;
    // } else {
    //     alpha_pos = 1 - (velocity.z() / 250);
    // }
    // position.z() = alpha_pos * baro->getAGLAltM() + (1-alpha_pos) * z_accel;
    // velocity.z() = alpha_velo * baroVelocity + (1-alpha_velo) * zdot_accel;
}

// Airbrake Functions from last year
// // Calculate Actuation Angle
int AirbrakeState::calculateActuationAngle(double altitude, double velocity, double tilt)
{

    int i = 0;
    // initial flap guesses
    double low = 0;
    double high = 65;
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
    astra::Vector<3> dragAccel = {acceleration.x(), acceleration.y(), acceleration.z()+9.81};
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
