%% main.m
% This script is the entrance point to the matlab state estimation for airbrake.
% This script is broken up into 3 parts. The first part is a section to
% read in data. This data can be generated or actual flight data. The
% second part is to define the state estiamtor used. And the last part is
% just code to determine how effective the filter is.


%% Section 1: Read in Data

% Generate Mock Data
fileName = 'MockData/mock_1.csv';
DataGenerator(fileName, 10, 100, .1, .5);
data = readtable(fileName);

% Read in OpenRocket File
% TODO

% Read in Flight Data File
% TODO



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

r_output_x = [];
r_output_y = [];
r_output_z = [];
v_output_x = [];
v_output_y = [];
v_output_z = [];

% Run filter
for i = 2:length(data.t)
    dt = data.t(i) - data.t(i - 1);
    measurement = [data.r_meas_x(i)];
    control = [data.r_meas_x(i); data.a_meas_y(i); data.a_meas_z(i) - 9.8];
    kf = kf.iterate(dt, measurement, control);
    r_output_x = [r_output_x, kf.X(1)];
    r_output_y = [r_output_y, kf.X(2)];
    r_output_z = [r_output_z, kf.X(3)];
    v_output_x = [v_output_x, kf.X(4)];
    v_output_y = [v_output_y, kf.X(5)];
    v_output_z = [v_output_z, kf.X(6)];
end

output = table(r_output_x, r_output_y, r_output_z, v_output_x, v_output_y, v_output_z);

%% Section 3: Analyize Output

% Z position vs time (Actual, Measured, Output)
figure;
plot(data.t, data.r_act_z, 'r-', 'DisplayName', 'Actual Z Position');
hold on;
plot(data.t, data.r_meas_z, 'g--', 'DisplayName', 'Measured Z Position');
plot(data.t(2:end), output.r_output_z, 'b-.', 'DisplayName', 'Output Z Position');
xlabel('Time (s)');
ylabel('Z Position (m)');
title('Z Position vs Time');
legend('show');
grid on;
hold off;

% Plot x, y, z positions vs time
figure;
subplot(3,1,1);
plot(data.t, data.r_meas_x, 'r', 'DisplayName', 'Measured x');
hold on;
plot(data.t, output.r_output_x, 'b', 'DisplayName', 'Output x');
xlabel('Time (s)');
ylabel('x Position (m)');
title('x Position vs Time');
legend show;
grid on;

subplot(3,1,2);
plot(data.t, data.r_meas_y, 'r', 'DisplayName', 'Measured y');
hold on;
plot(data.t, output.r_output_y, 'b', 'DisplayName', 'Output y');
xlabel('Time (s)');
ylabel('y Position (m)');
title('y Position vs Time');
legend show;
grid on;

subplot(3,1,3);
plot(data.t, data.r_meas_z, 'r', 'DisplayName', 'Measured z');
hold on;
plot(data.t, output.r_output_z, 'b', 'DisplayName', 'Output z');
xlabel('Time (s)');
ylabel('z Position (m)');
title('z Position vs Time');
legend show;
grid on;

% Plot z velocity vs time
figure;
plot(data.t, data.v_meas_z, 'r', 'DisplayName', 'Measured z Velocity');
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
plot(data.t, data.v_meas_x, 'r', 'DisplayName', 'Measured x Velocity');
hold on;
plot(data.t, output.v_output_x, 'b', 'DisplayName', 'Output x Velocity');
xlabel('Time (s)');
ylabel('x Velocity (m/s)');
title('x Velocity vs Time');
legend show;
grid on;

subplot(3,1,2);
plot(data.t, data.v_meas_y, 'r', 'DisplayName', 'Measured y Velocity');
hold on;
plot(data.t, output.v_output_y, 'b', 'DisplayName', 'Output y Velocity');
xlabel('Time (s)');
ylabel('y Velocity (m/s)');
title('y Velocity vs Time');
legend show;
grid on;

subplot(3,1,3);
plot(data.t, data.v_meas_z, 'r', 'DisplayName', 'Measured z Velocity');
hold on;
plot(data.t, output.v_output_z, 'b', 'DisplayName', 'Output z Velocity');
xlabel('Time (s)');
ylabel('z Velocity (m/s)');
title('z Velocity vs Time');
legend show;
grid on;