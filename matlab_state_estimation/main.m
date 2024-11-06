%% main.m
% This script is the entrance point to the matlab state estimation for airbrake.
% This script is broken up into 3 parts. The first part is a section to
% read in data. This data can be generated or actual flight data. The
% second part is to define the state estiamtor used. And the last part is
% just code to determine how effective the filter is.

clear
clc

%% Section 0: Set Simulation Parameters
% Set this var to where the data will come from [Mock, OpenRocket, Flight]
dataType = DataType.Mock;
mockDataFile = 'mock_1.csv';
openRocketDataFile = 'openrocket_2024_30k.csv';
flightDataFile = 'TADPOL_April_NY_post_processed2.csv';

% Set sensor gaussian noise
accelNoise = .1; % standard deviation w/ units m/s^2
baroNoise = .5; % standard deviation w/ units m
gpsNoise = 1; % standard deviation w/ units m
g = 9.81; % m/s^2


%% Section 1: Read in Data

% Create mock rocket
if dataType == DataType.Mock
    rocketMotorAccel = 125; % m/s^2
    rocketMotorBurnTime = 5; % seconds
    rocketDragCoef = .5;
    rocketCrossSectionalArea = 0.1524*0.1524*pi; % 6in diameter in m^2 
    rocket = rocket(rocketMotorAccel, rocketMotorBurnTime, rocketDragCoef, rocketCrossSectionalArea);
    
    % Generate Mock Data
    mockDataFolder = 'MockData';
    fileName = [mockDataFolder '/' mockDataFile];
    loopFrequency = 50; % in Hz
    DataGenerator(fileName, loopFrequency, rocket)
    data = readtable(fileName);
end

% Read in Open Rocket Simulation File
if dataType == DataType.OpenRocket
    openRocketDataFolder = 'OpenRocketData';
    fileName = [openRocketDataFolder '/' openRocketDataFile];
    dataraw = readtable(fileName);
    data.t = dataraw.x_Time_s_;
    data.r_x = dataraw.PositionEastOfLaunch_m_;
    data.r_y = dataraw.PositionNorthOfLaunch_m_;
    data.r_z = dataraw.Altitude_m_;
    
    % Diff position to get velocity
    delta_t = diff(data.t);
    delta_r_x = diff(data.r_x);
    delta_r_y = diff(data.r_y);
    
    data.v_x = [delta_r_x ./ delta_t; 0];
    data.v_y = [delta_r_y ./ delta_t; 0];
    data.v_z = dataraw.VerticalVelocity_m_s_;
    
    % Diff velocity to get acceleration
    delta_v_x = diff(data.v_x);
    delta_v_y = diff(data.v_y);
    
    data.a_x = [delta_v_x ./ delta_t; 0];
    data.a_y = [delta_v_y ./ delta_t; 0];
    data.a_z = dataraw.VerticalAcceleration_m_s__;
end

% Read in Flight Data File
if dataType == DataType.Flight
    flightDataFolder = 'FlightData';
    fileName = [flightDataFolder '/' flightDataFile];
    dataraw = readtable(fileName);
    
    data.t = dataraw.Time_ms_;
    data.r_x = zeros(size(data.t)); data.r_y = zeros(size(data.t)); data.r_z = zeros(size(data.t));
    data.v_x = zeros(size(data.t)); data.v_y = zeros(size(data.t)); data.v_z = zeros(size(data.t));
    data.a_x = zeros(size(data.t)); data.a_y = zeros(size(data.t)); data.a_z = zeros(size(data.t));
    data.r_meas_x = dataraw.GPSPosX;
    data.r_meas_y = dataraw.GPSPosY;
    data.r_meas_z = dataraw.BarAltitude;
    data.a_meas_x = dataraw.IMUAccelX;
    data.a_meas_y = dataraw.IMUAccelY;
    data.a_meas_z = dataraw.IMUAccelZ;
end

% 1.1: Add gaussian noise to generate measurement data
if dataType ~= DataType.Flight
    data.r_meas_x = GaussianNoiseGenerator(data.r_x, gpsNoise);
    data.r_meas_y = GaussianNoiseGenerator(data.r_y, gpsNoise);
    data.r_meas_z = GaussianNoiseGenerator(data.r_z, baroNoise);
    data.a_meas_x = GaussianNoiseGenerator(data.a_x, accelNoise);
    data.a_meas_y = GaussianNoiseGenerator(data.a_y, accelNoise);
    data.a_meas_z = GaussianNoiseGenerator(data.a_z + 2*g, accelNoise); % Accelerameters at rest read +g in z-axis (when we mock data, including openrocket) it outputs -g in z-axis at rest
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
initial_dt = data.t(2) - data.t(1);
kf = LinearKalmanFilter(initial_state, P, initial_control, initial_dt);

r_output_x = [kf.X(1)];
r_output_y = [kf.X(2)];
r_output_z = [kf.X(3)];
v_output_x = [kf.X(4)];
v_output_y = [kf.X(5)];
v_output_z = [kf.X(6)];

% Run filter
for i = 2:length(data.t)
    dt = data.t(i) - data.t(i - 1);
    measurement = [data.r_meas_x(i); data.r_meas_y(i); data.r_meas_z(i)];
    control = [data.a_meas_x(i); data.a_meas_y(i); data.a_meas_z(i) - g]; % Accelerameters at rest read +g in z-axis but are at rest
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
xlabel('Time (s)', 'FontSize', 16);
ylabel('Z Position (m)', 'FontSize', 16);
title('Z Position vs Time', 'FontSize', 18);
legend('show', 'FontSize', 14);
grid on;
hold off;

% Plot x, y, z positions vs time
figure;
subplot(3,1,1);
plot(data.t, data.r_x, 'r', 'DisplayName', 'Actual x');
hold on;
plot(data.t, data.r_meas_x, 'g.', 'DisplayName', 'Measured X Position');
plot(data.t, output.r_output_x, 'b', 'DisplayName', 'Output x');
xlabel('Time (s)');
ylabel('x Position (m)');
title('x Position vs Time');
legend show;
grid on;

subplot(3,1,2);
plot(data.t, data.r_y, 'r', 'DisplayName', 'Actual y');
plot(data.t, data.r_meas_y, 'g.', 'DisplayName', 'Measured Y Position');
plot(data.t, output.r_output_y, 'b', 'DisplayName', 'Output y');
xlabel('Time (s)');
ylabel('y Position (m)');
title('y Position vs Time');
legend show;
grid on;

subplot(3,1,3);
plot(data.t, data.r_z, 'r', 'DisplayName', 'Actual z');
plot(data.t, data.r_meas_z, 'g.', 'DisplayName', 'Measured Z Position');
plot(data.t, output.r_output_z, 'b', 'DisplayName', 'Output z');
xlabel('Time (s)');
ylabel('z Position (m)');
title('z Position vs Time');
legend show;
grid on;
hold off;

% Plot z velocity vs time
figure;
plot(data.t, data.v_z, 'r', 'DisplayName', 'Actual z Velocity');
hold on;
plot(data.t, output.v_output_z, 'b', 'DisplayName', 'Output z Velocity');
xlabel('Time (s)', 'FontSize', 16);
ylabel('z Velocity (m/s)', 'FontSize', 16);
title('z Velocity vs Time', 'FontSize', 18);
legend('show', 'FontSize', 14);
grid on;
hold off;

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
plot(data.t, output.v_output_y, 'b', 'DisplayName', 'Output y Velocity');
xlabel('Time (s)');
ylabel('y Velocity (m/s)');
title('y Velocity vs Time');
legend show;
grid on;

subplot(3,1,3);
plot(data.t, data.v_z, 'r', 'DisplayName', 'Actual z Velocity');
plot(data.t, output.v_output_z, 'b', 'DisplayName', 'Output z Velocity');
xlabel('Time (s)');
ylabel('z Velocity (m/s)');
title('z Velocity vs Time');
legend show;
grid on;
hold off;