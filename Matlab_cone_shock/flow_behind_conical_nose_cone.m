clear;clc;
addpath('Taylor Maccoll Solution for Supersonic Flow over a Cone')

% Much of this code is based on John D. Anderson Jr. - Fundamentals of Aerodynamics-McGraw Hill
% Oblique Shocks - Chapter 9.2

% Rocket height properies
height_rocket = 13; % in ft
height_fin_can = 3; % in ft
height_airbrake = 1; % in ft
diameter_rocket = .5; % in ft

% Rocket nose cone properies
height_nose_cone = 3; % in ft
diameter_nose_cone = diameter_rocket;
cone_angle = atand(diameter_nose_cone/(2*height_nose_cone))

% Airbrake properies
length_of_flaps = .5; % in ft
airbrake_flap_angle = 12; % in deg
flap_area = 0.0064516; % in m^2
torque_arm = 1/6; % in ft

% Flow input parameters
gamma = 1.4;
M_inf = 1.6;
p_inf = 76000; % in Pa
T_inf = 45 + 273.15; % in K
R = 287; % universal gas constant p=rho*R*T in J/kg*K
rho_inf = p_inf/(R*T_inf);

% First shock
% 3D cone shock
beta_nose = shock_angle(M_inf, cone_angle, gamma)
Minf_n = M_inf*sind(beta_nose)
M1_n = sqrt((1 + Minf_n*Minf_n*(gamma-1)/2)/(gamma*Minf_n*Minf_n - ((gamma-1)/2)))
M1 = M1_n/(sind(beta_nose-cone_angle))
p1 = p_inf*(1 + (2*gamma)*(Minf_n*Minf_n - 1)/(gamma + 1));
rho1 = rho_inf*((Minf_n*Minf_n*(gamma+1))/(2 + (gamma-1)*Minf_n*Minf_n));
T1 = T_inf*p_inf*rho_inf/(p1*rho1);


% Second shock
% 2D wedge shock
beta_flap = wedge_2d_shock_angle(M1, airbrake_flap_angle, gamma, 0)
M2inf_n = M1*sind(beta_flap)
M2_n = sqrt((1 + M2inf_n*M2inf_n*(gamma-1)/2)/(gamma*M2inf_n*M2inf_n - ((gamma-1)/2)))
M2 = M2_n/(sind(beta_flap-airbrake_flap_angle))
p2 = p1*(1 + (2*gamma)*(M2inf_n*M2inf_n - 1)/(gamma + 1));
rho2 = rho1*((Minf_n*Minf_n*(gamma+1))/(2 + (gamma-1)*M2inf_n*M2inf_n));

q = .5*rho2*(343*M2)^2
F = q*flap_area*sind(airbrake_flap_angle)
torque = F*1/6;
force_on_disc = torque/(1/4)

% Visualize the shock and rocket
figure;
hold on;
axis equal;
xlabel('Length (ft)');
ylabel('Height (ft)');

% Plot the rocket body
rectangle('Position', [0, 0, diameter_rocket, height_rocket-height_nose_cone], 'FaceColor', [0.5 0.5 0.5]);

% Plot the nose cone
nose_cone_x = [0, diameter_nose_cone / 2, diameter_nose_cone];
nose_cone_y = [height_rocket-height_nose_cone, height_rocket, height_rocket-height_nose_cone];
fill(nose_cone_x, nose_cone_y, [0.7 0.7 0.7]);

% Plot the airbrake
airbrake_x = [diameter_rocket, diameter_rocket + length_of_flaps, diameter_rocket];
airbrake_y = [height_fin_can + height_airbrake, height_fin_can + height_airbrake - length_of_flaps, height_fin_can + height_airbrake];
fill(airbrake_x, airbrake_y, [0.8 0.8 0.8]);
airbrake_x = [0, -length_of_flaps, 0];
airbrake_y = [height_fin_can + height_airbrake, height_fin_can + height_airbrake - length_of_flaps, height_fin_can + height_airbrake];
fill(airbrake_x, airbrake_y, [0.8 0.8 0.8]);

% Plot the shock angle
shock_x = [diameter_nose_cone/2, diameter_nose_cone/2 + height_rocket * sind(beta_nose)];
shock_y = [height_rocket + height_nose_cone, 0];
plot(shock_x, shock_y, 'r');
shock_x = [diameter_nose_cone/2, -(diameter_nose_cone/2 + height_rocket * sind(beta_nose))];
shock_y = [height_rocket + height_nose_cone, 0];
plot(shock_x, shock_y, 'r');

title(sprintf('Rocket with Shock Angle: %.2f°', beta_nose));
hold off;