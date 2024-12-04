#include <Arduino.h>

#include "airbrake_state.h"

void AirbrakeState::updateState(double newTime) {
    mmfs::State::updateState(newTime); // call base version for sensor updates
    setAirbrakeStage();
}

void AirbrakeState::setAirbrakeStage(){
    // TODO implement this
}


// Airbrake Functions from last year
// // Calculate Actuation Angle
// int AirbrakeState::calculateActuationAngle(double altitude, double velocity, double tilt, double loop_time) { 

//     int i = 0;
//     // initial flap guesses
//     double low = 0; 
//     double high = 90;
            
//     while (i < max_guesses) {
 
//         double sim_apogee = predict_apogee(loop_time, cda_rocket, actuationAngle, tilt, velocity, altitude, ground_altitude, empty_mass); 
//         double apogee_difference = estimated_apogee - target_apogee;

//          if(estimated_apogee != estimated_apogee){ // TODO why does isnan(estimated_apogee) not work?
//           actuationAngle = 0;
//           break;
//          }
        
//         if (abs(apogee_difference) < threshold) {
//             break;
//         } else if (apogee_difference > 0) {
//             low = actuationAngle;
//         } else if (apogee_difference < 0) {
//             high = actuationAngle;
//         }
        
//         actuationAngle = (high + low) / 2;
//         i++;
//     }
    
//     // sets angles in degrees
//     actuationAngle = angle_resolution*round(actuationAngle/angle_resolution);
// }

// // Calculate apogee
// double AirbrakeState::predict_apogee(double time_step, double CdA_rocket, double flap_angle, double tilt, double cur_velocity, double cur_height, double ground_alt, double empty_mass) {
    
//     double time_intergrating = 0.0;
//     double x = 0.0;
//     double dx = sin(tilt)*cur_velocity;
//     double ddx = 0.0;
//     double y = cur_height;
//     double dy = cos(tilt)*cur_velocity;
//     double ddy = 0.0;
//     double k1x = 0.0;
//     double k1y = 0.0;
//     double s1x = 0.0;
//     double s1y = 0.0;
//     double k2x = 0.0;
//     double k2y = 0.0;
//     double CdA_flaps = 4*0.95*0.00645*sin(flap_angle*3.141592/180);

//     while (time_intergrating < 60){
//     k1x = -0.5*get_density(y+ground_alt)*(CdA_rocket+CdA_flaps)*sqrt(dx*dx+dy*dy)*dx/empty_mass;
//     k1y = -0.5*get_density(y+ground_alt)*(CdA_rocket+CdA_flaps)*sqrt(dx*dx+dy*dy)*dy/empty_mass - 9.81;

//     s1x = dx + (time_step * k1x);
//     s1y = dy + (time_step * k1y);

//     k2x = -0.5*get_density(y+ground_alt)*(CdA_rocket+CdA_flaps)*sqrt(pow(s1x,2)+pow(s1y,2))*(s1x)/empty_mass;
//     k2y = -0.5*get_density(y+ground_alt)*(CdA_rocket+CdA_flaps)*sqrt(pow(s1x,2)+pow(s1y,2))*(s1y)/empty_mass - 9.81;

//     dx += time_step*(k1x+k2x)/2;
//     dy += time_step*(k1y+k2y)/2;
//     x += time_step*dx;
//     y += time_step*dy;
//     time_intergrating += time_step;

//     if (dy <= 0){
//     return y;
//     }
    
//     }
//     return y; 
// }

// // Calculate air density
// double AirbrakeState::get_density(double h){
//     // Constants
//     double R=8.31446;  //universal gas constant (J/(mol·K))
//     double M=.0289652; //molar mass of air (kg/mol)
//     double L=0.0065;  //temperature lapse rate in the troposphere (K/m)

//     double p0=101325; //ground pressure (Pa) // TODO update. get from baro in MMFS
//     double T0=288.15; //ground temperature (K) // TODO update. get from baro in MMFS

  
//     density = p0*M/R/T0*pow((1-L*h/T0),((9.8*M/R/L)-1));
  
// }

// // estimate CdA
// void AirbrakeState::update_cda_estimate() {
//     cda_number_of_measurements++;
//     double CdA_rocket_this_time_step =  (2*empty_mass*abs(acceleration.z()))/(density*velocity.z()*velocity.z()); // TODO I think this wrong, needs to be body frame accel and velo
//     cda_rocket = (  cda_rocket*(cda_number_of_measurements-1) +  CdA_rocket_this_time_step    )/cda_number_of_measurements ;  
//     logger.recordLogData(mmfs::INFO_, 'CdA of the Rocket is ' + String(cda_rocket).c_str());
// }