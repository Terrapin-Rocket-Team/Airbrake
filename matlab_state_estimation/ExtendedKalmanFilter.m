
classdef ExtendedKalmanFilter
    properties
        F % State Transition Matrix
        G % Control Matrix
        U % Control Vector
        X % State Vector
        H % Observation Matrix
        P % Estimate Covariance Matrix
        R % Measurement Uncertainty Matrix
        K % Kalman Gain
        Q % Process Noise Matrix
        c % Drag thing
        
        % Mass Stuff
        m_wet % wet mass (kg)
        m_dry % dry mass (kg)
        t_b % burn time
        stage % 0, 1, 2 [prelaunch, boost, post boost]
        dt % change in time [s]
        m % object current mass (kg)
        g = 9.81 % earth acceleration due to gravity [m/s^2]

        % Orientation Stuff
        tilt % in degrees DCM 3x3
        yaw % in degrees DCM 1x1

        process_noise = .1; % [m/s^2]
    end

    methods
        % Constructor
        function obj = ExtendedKalmanFilter(X, P, dt, wet_mass, dry_mass, burn_time)
            if nargin ~= 6 && (~ismatrix(X) || ~ismatrix(P))
                error('Incorrect amount of arguments passed in or incorrect arg formats.');
            end

            obj.X = X;
            obj.P = P;

            obj.R = [1.^2, 0, 0, 0, 0, 0, 0, 0;
                     0, 1.^2, 0, 0, 0, 0, 0, 0;
                     0, 0, 1.^2, 0, 0, 0, 0, 0;
                     0, 0, 0, .2.^2, 0, 0, 0, 0;
                     0, 0, 0, 0, .5.^2, 0, 0, 0;
                     0, 0, 0, 0, 0, .1.^2, 0, 0;
                     0, 0, 0, 0, 0, 0, .1.^2, 0;
                     0, 0, 0, 0, 0, 0, 0, .1.^2;];

            obj.m_wet = wet_mass;
            obj.m_dry = dry_mass;
            obj.t_b = burn_time;
            obj = obj.calculateInitialValues(dt, 0, 0, 0);
        end

        function obj = predictState(obj)
            a = obj.tilt;
            b = obj.yaw;
            D = getD(obj);

            obj.X = [obj.X(4);
                     obj.X(5);
                     obj.X(6);
                     (D/obj.m)*sin(a)*cos(b);
                     (D/obj.m)*sin(a)*cos(b);
                     (D/obj.m)*cos(b) - obj.g/obj.m
                ];
        end
        
        function obj = updateState(obj, measurement)
            obj.X = obj.X + obj.K*(measurement - measurementFun(obj.X));
        end

        function obj = calculateKalmanGain(obj)
            obj.K = obj.P * obj.H' * inv(obj.H * obj.P * obj.H' + obj.R);
        end

        function obj = covarianceUpdate(obj)
            n = size(obj.P, 1);
            obj.P = (eye(n) - obj.K*obj.H)*obj.P*(eye(n) - obj.K*obj.H)' + obj.K*obj.R*obj.K';
        end

        function obj = covarianceExtrapolate(obj)
            obj.P = obj.F*obj.P*obj.F'+ obj.Q;
        end
        
        function obj = calculateInitialValues(obj, dt, stage, tilt, yaw)
            
            obj.Q = [(dt^4)/4, 0, 0, (dt^3)/2, 0, 0;
                     0, (dt^4)/4, 0, 0, (dt^3)/2, 0;
                     0, 0, (dt^4)/4, 0, 0, (dt^3)/2;
                     (dt^3)/2, 0, dt^2, 0, 0, 0;
                     0, (dt^3)/2, 0, 0, dt^2, 0;
                     0, 0, (dt^3)/2, 0, 0, dt^2]*obj.process_noise*obj.process_noise;
            
            % Kalman Filter Steps
            obj.dt = dt;
            obj.stage = stage;
            obj.tilt = tilt;
            obj.yaw = yaw;
            obj = obj.calcMass();

            % Dyanmics prediction
            obj = calcF(obj, dt);
            obj = predictState(obj);
            obj = covarianceExtrapolate(obj);
        end

        function obj = iterate(obj, dt, measurement, control, stage, tilt, yaw)
            % Update step
            obj.U = control;

            obj.F =    [1, 0, 0, dt, 0, 0;
                        0, 1, 0, 0, dt, 0;
                        0, 0, 1, 0, 0, dt;
                        0, 0, 0, 1, 0,  0;
                        0, 0, 0, 0, 1,  0;
                        0, 0, 0, 0, 0,  1];
        
            obj.G = [0.5*dt*dt, 0, 0;
                     0, 0.5*dt*dt, 0;
                     0, 0, 0.5*dt*dt;
                     dt, 0,        0;
                     0, dt,        0;
                     0, 0,       dt];
        
            % Recalculate Q based on updated G
            obj.Q = obj.G * obj.process_noise^2 * obj.G';
        
            % Kalman Filter Steps
            obj.dt = dt;
            obj.stage = stage;
            obj.tilt = tilt;
            obj.yaw = yaw;
            obj = obj.calcMass();

            % Measurement Update
            obj = calcH(obj);
            obj = calculateKalmanGain(obj);
            obj = updateState(obj, measurement);
            obj = covarianceUpdate(obj);

            % Dyanmics prediction
            obj = calcF(obj, dt);
            obj = predictState(obj);
            obj = covarianceExtrapolate(obj);
        end

        function obj = calcF(obj, dt)
            a = obj.tilt;
            b = obj.yaw;
            D = getD(obj);

            A = [
                0, 0, 0, 1, 0, 0;
                0, 0, 0, 0, 1, 0;
                0, 0, 0, 0, 0, 1;
                0, 0, 0, -obj.X(4)*D*sind(a)*cosd(b), -obj.X(5)*D*sind(a)*cosd(b), -obj.X(6)*D*sind(a)*cosd(b);
                0, 0, 0, -obj.X(4)*D*sind(a)*sind(b), -obj.X(5)*D*sind(a)*sind(b), -obj.X(6)*D*sind(a)*sind(b);
                0, 0, 0, -obj.X(4)*D*cosd(a), -obj.X(5)*D*cosd(a), -obj.X(6)*D*cosd(a);
                ];

            obj.F = expm(A*dt);
        end

        function obj = calcH(obj)
            a = obj.tilt;
            b = obj.yaw;
            D = getD(obj);

            obj.H = [
                    1, 0, 0, 0, 0, 0;
                    0, 1, 0, 0, 0, 0;
                    0, 0, 1, 0, 0, 0;
                    0, 0, 1, 0, 0, 0;
                    0, 0, 1, 0, 0, 0;
                    0, 0, 0, -obj.X(4)*D*sind(a)*cosd(b), -obj.X(5)*D*sind(a)*cosd(b), -obj.X(6)*D*sind(a)*cosd(b);
                    0, 0, 0, -obj.X(4)*D*sind(a)*sind(b), -obj.X(5)*D*sind(a)*sind(b), -obj.X(6)*D*sind(a)*sind(b);
                    0, 0, 0, -obj.X(4)*D*cosd(a), -obj.X(5)*D*cosd(a), -obj.X(6)*D*cosd(a);
                    ];
        end

        function Hx = measurementFun(X)
            a = obj.tilt;
            b = obj.yaw;
            D = getD(obj);

            Hx = [
                X(1);
                X(2);
                X(3);
                X(3);
                X(3);
                -1/2 * D * X(4) * X(4) * sind(a) * cosd(b);
                -1/2 * D * X(5) * X(5) * sind(a) * sind(b);
                -1/2 * D * X(6) * X(6) * cosd(a);
            ]; 
        end

        function D = getD(obj)
            area = pi * (6/(2*39.37)).^2; % 6 diameter to m then A=pi*r^2
            rho = 1.225; % sea level denisty kg/m3
            Cd = .5; % coeffient of drag

            D = rho * Cd * area / obj.m;

        end

        function obj = calcMass(obj)
            if(obj.stage == 0)
                obj.m = obj.m_wet;
            elseif(obj.stage == 1)
                obj.m = obj.m - ((obj.m_wet - obj.m_dry)/obj.t_b)*obj.dt;
            else
                obj.m = obj.m_dry;
            end
        end
    end
end


