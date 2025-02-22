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
flightDataFile = 'avionics_12_8_24_post.csv';

filterType = FilterType.EKF;

% Set sensor gaussian noise
accelNoise = .1; % standard deviation w/ units m/s^2
baro1Noise = .2; % standard deviation w/ units m
baro2Noise = .5; % standard deviation w/ units m
gpsNoise = 1; % standard deviation w/ units m
g = 9.81; % m/s^2


%% Section 1: Read in Data

% Create mock rocket
if dataType == DataType.Mock
    rocketTotalImpulse = 32000; % [Ns]
    rocketDryMass = 39; % kg
    rocketWetMass = 56.25; % kg
    rocketMotorBurnTime = 4.5; % seconds
    rocketDragCoef = .5;
    rocketCrossSectionalArea = 0.0762*0.0762*pi; % 6in diameter in m^2 
    rocket = rocket(rocketTotalImpulse, rocketWetMass, rocketDryMass, rocketMotorBurnTime, rocketDragCoef, rocketCrossSectionalArea);
    tilt = 0;
    yaw = 0;
    
    % Generate Mock Data
    mockDataFolder = 'MockData';
    fileName = [mockDataFolder '/' mockDataFile];
    loopFrequency = 25; % in Hz
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
    data.r_meas1_z = dataraw.GPSPosZ;
    data.r_meas2_z = dataraw.BarAltitude;
    data.r_meas3_z = dataraw.BarAltitude2;
    data.a_meas_x = dataraw.IMUAccelX;
    data.a_meas_y = dataraw.IMUAccelY;
    data.a_meas_z = dataraw.IMUAccelZ;
end

% 1.1: Add gaussian noise to generate measurement data
if dataType ~= DataType.Flight
    data.r_meas_x = GaussianNoiseGenerator(data.r_x, gpsNoise);
    data.r_meas_y = GaussianNoiseGenerator(data.r_y, gpsNoise);
    data.r_meas1_z = GaussianNoiseGenerator(data.r_z, gpsNoise);
    data.r_meas2_z = GaussianNoiseGenerator(data.r_z, baro1Noise);
    data.r_meas3_z = GaussianNoiseGenerator(data.r_z, baro2Noise);
    data.a_meas_x = GaussianNoiseGenerator(data.a_x, accelNoise);
    data.a_meas_y = GaussianNoiseGenerator(data.a_y, accelNoise);
    data.a_meas_z = GaussianNoiseGenerator(data.a_z + 2*g, accelNoise); % Accelerameters at rest read +g in z-axis (when we mock data, including openrocket) it outputs -g in z-axis at rest
end


%% Section 2: Run Filter

% Initalize the filter
initial_control = [0; 0; 0];
initial_state = [0; 0; 0; 0; 0; 0];
P = 500 * [1 0 0 0 0 0;
           0 1 0 0 0 0;
           0 0 1 0 0 0;
           0 0 0 1 0 0;
           0 0 0 0 1 0;
           0 0 0 0 0 1];
initial_dt = data.t(2) - data.t(1);

if(filterType == FilterType.LKF)
    kf = LinearKalmanFilterMoreMeasurements(initial_state, P, initial_control, initial_dt);
elseif (filterType == FilterType.LKFMM)
    kf = LinearKalmanFilterMoreMeasurements(initial_state, P, initial_control, initial_dt);
elseif (filterType == FilterType.EKF)
    kf = ExtendedKalmanFilter(initial_state, P, initial_dt, rocketWetMass, rocketWetMass, rocketMotorBurnTime);
end

r_output_x = [kf.X(1)];
r_output_y = [kf.X(2)];
r_output_z = [kf.X(3)];
v_output_x = [kf.X(4)];
v_output_y = [kf.X(5)];
v_output_z = [kf.X(6)];
P_output = [P];

% Run filter
stage = 1;
for i = 2:length(data.t)
    dt = data.t(i) - data.t(i - 1);
    control = [data.a_meas_x(i); data.a_meas_y(i); data.a_meas_z(i) - g]; % Accelerameters at rest read +g in z-axis but are at rest
    if (stage == 1) && (data.a_meas_z(i) < 0)
        stage = 2;
    end
    if (filterType == FilterType.EKF)
        measurement = [data.r_meas_x(i); data.r_meas_y(i); data.r_meas1_z(i); data.r_meas2_z(i); data.r_meas3_z(i); data.a_meas_x(i); data.a_meas_y(i); data.a_meas_z(i)-g];
        kf = kf.iterate(dt, measurement, control, stage, 0, 0);
    else
        measurement = [data.r_meas_x(i); data.r_meas_y(i); data.r_meas1_z(i); data.r_meas2_z(i); data.r_meas3_z(i)];
        kf = kf.iterate(dt, measurement, control);
    end
    r_output_x = [r_output_x; kf.X(1)];
    r_output_y = [r_output_y; kf.X(2)];
    r_output_z = [r_output_z; kf.X(3)];
    v_output_x = [v_output_x; kf.X(4)];
    v_output_y = [v_output_y; kf.X(5)];
    v_output_z = [v_output_z; kf.X(6)];
    P_output = cat(3, P_output, kf.P);
end

X_output = table(r_output_x, r_output_y, r_output_z, v_output_x, v_output_y, v_output_z);

%% Section 3: Analyize Output

plotFilterResults(dataType, data, X_output, P_output)