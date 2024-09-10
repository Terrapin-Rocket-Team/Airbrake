%% main.m
% This script is the entrance point to the matlab state estimation for airbrake.
% This script is broken up into 3 parts. The first part is a section to
% read in data. This data can be generated or actual flight data. The
% second part is to define the state estiamtor used. And the last part is
% just code to determine how effective the filter is.

clear
clc

%% Section 1: Read in Data
dataType = DataType.OpenRocket;


% Create mock rocket
if dataType == DataType.Mock
rocketMotorAccel = 100; % m/s^2
rocketMotorBurnTime = 1; % seconds
rocketDragCoef = .5;
rocketCrossSectionalArea = 0.1524*0.1524*pi; % 6in diameter in m^2 
rocket = rocket(rocketMotorAccel, rocketMotorBurnTime, rocketDragCoef, rocketCrossSectionalArea);

% Generate Mock Data
fileName = 'MockData/mock_1.csv';
loopFrequency = 50; % in Hz
accelNoise = .1; % standard deviation w/ units m/s^2
baroNoise = .5; % standard deviation w/ units m
DataGenerator(fileName, 50, accelNoise, baroNoise, rocket)
data = readtable(fileName);
end

% Read in Open Rocket Simulation File
if dataType == DataType.OpenRocket
accelNoise = .1; % standard deviation w/ units m/s^2
baroNoise = .5; % standard deviation w/ units m

fileName = 'OpenRocketData/openrocket_test_data.csv';
dataraw = readtable(fileName);
data.t = dataraw.x_Time_s_;
data.r_x = dataraw.PositionEastOfLaunch_m_;
data.r_y = dataraw.PositionNorthOfLaunch_m_;
data.r_z = dataraw.Altitude_m_;

delta_t = diff(data.t);
delta_r_x = diff(data.r_x);
delta_r_y = diff(data.r_y);

data.v_x = [delta_r_x ./ delta_t; 0];
data.v_y = [delta_r_y ./ delta_t; 0];
data.v_z = dataraw.VerticalVelocity_m_s_;

delta_v_x = diff(data.v_x);
delta_v_y = diff(data.v_y);

data.a_x = [delta_v_x ./ delta_t; 0];
data.a_y = [delta_v_y ./ delta_t; 0];
data.a_z = dataraw.VerticalAcceleration_m_s__;
data.r_meas_x = GaussianNoiseGenerator(data.r_x, baroNoise);
data.r_meas_y = GaussianNoiseGenerator(data.r_y, baroNoise);
data.r_meas_z = GaussianNoiseGenerator(data.r_z, baroNoise);
data.a_meas_x = GaussianNoiseGenerator(data.a_x, accelNoise);
data.a_meas_y = GaussianNoiseGenerator(data.a_y, accelNoise);
data.a_meas_z = GaussianNoiseGenerator(data.a_z, accelNoise);
end

% Read in Flight Data File
if dataType == DataType.Flight

end



%% Section 2: Run Filter

% Initalize the filter
initial_control = [0; 0; 0];
initial_state = [0; 0; 0; 0; 0; 0];
P = 500 * [1 0 0 1 0 0;
           0 1 0 0 1 0;
           0 0 1 0 0 1;
           1 0 0 1 0 0;
           0 1 0 0 1 0;
           0 0 1 0 0 1];
kf = LinearKalmanFilter(initial_state, P, initial_control);

r_output_x = [kf.X(1)];
r_output_y = [kf.X(2)];
r_output_z = [kf.X(3)];
v_output_x = [kf.X(4)];
v_output_y = [kf.X(5)];
v_output_z = [kf.X(6)];

% Run filter
for i = 2:length(data.t)
    dt = data.t(i) - data.t(i - 1);
    measurement = [data.r_meas_z(i)];
    control = [data.a_meas_x(i); data.a_meas_y(i); data.a_meas_z(i) - 9.8];
    kf = kf.iterate(dt, measurement, control);
    r_output_x = [r_output_x; kf.X(1)];
    r_output_y = [r_output_y; kf.X(2)];
    r_output_z = [r_output_z; kf.X(3)];
    v_output_x = [v_output_x; kf.X(4)];
    v_output_y = [v_output_y; kf.X(5)];
    v_output_z = [v_output_z; kf.X(6)];
end

output = table(r_output_x, r_output_y, r_output_z, v_output_x, v_output_y, v_output_z);

%% Section 3: Analyize Output

% Z position vs time (Actual, Measured, Output)
figure;
plot(data.t, data.r_z, 'r-', 'DisplayName', 'Actual Z Position');
hold on;
plot(data.t, data.r_meas_z, 'g.', 'DisplayName', 'Measured Z Position');
plot(data.t, output.r_output_z, 'b', 'DisplayName', 'Output Z Position');
xlabel('Time (s)');
ylabel('Z Position (m)');
title('Z Position vs Time');
legend('show');
grid on;
hold off;

% Plot x, y, z positions vs time
figure;
subplot(3,1,1);
plot(data.t, data.r_x, 'r', 'DisplayName', 'Actual x');
hold on;
plot(data.t, output.r_output_x, 'b', 'DisplayName', 'Output x');
xlabel('Time (s)');
ylabel('x Position (m)');
title('x Position vs Time');
legend show;
grid on;

subplot(3,1,2);
plot(data.t, data.r_y, 'r', 'DisplayName', 'Actual y');
hold on;
plot(data.t, output.r_output_y, 'b', 'DisplayName', 'Output y');
xlabel('Time (s)');
ylabel('y Position (m)');
title('y Position vs Time');
legend show;
grid on;

subplot(3,1,3);
plot(data.t, data.r_z, 'r', 'DisplayName', 'Actual z');
hold on;
plot(data.t, output.r_output_z, 'b', 'DisplayName', 'Output z');
xlabel('Time (s)');
ylabel('z Position (m)');
title('z Position vs Time');
legend show;
grid on;

% Plot z velocity vs time
figure;
plot(data.t, data.v_z, 'r', 'DisplayName', 'Actual z Velocity');
hold on;
plot(data.t, output.v_output_z, 'b', 'DisplayName', 'Output z Velocity');
xlabel('Time (s)');
ylabel('z Velocity (m/s)');
title('z Velocity vs Time');
legend show;
grid on;

% Plot x, y, z velocities vs time
figure;
subplot(3,1,1);
plot(data.t, data.v_x, 'r', 'DisplayName', 'Actual x Velocity');
hold on;
plot(data.t, output.v_output_x, 'b', 'DisplayName', 'Output x Velocity');
xlabel('Time (s)');
ylabel('x Velocity (m/s)');
title('x Velocity vs Time');
legend show;
grid on;

subplot(3,1,2);
plot(data.t, data.v_y, 'r', 'DisplayName', 'Actual y Velocity');
hold on;
plot(data.t, output.v_output_y, 'b', 'DisplayName', 'Output y Velocity');
xlabel('Time (s)');
ylabel('y Velocity (m/s)');
title('y Velocity vs Time');
legend show;
grid on;

subplot(3,1,3);
plot(data.t, data.v_z, 'r', 'DisplayName', 'Actual z Velocity');
hold on;
plot(data.t, output.v_output_z, 'b', 'DisplayName', 'Output z Velocity');
xlabel('Time (s)');
ylabel('z Velocity (m/s)');
title('z Velocity vs Time');
legend show;
grid on;