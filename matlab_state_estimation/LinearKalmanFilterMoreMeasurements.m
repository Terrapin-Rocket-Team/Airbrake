
classdef LinearKalmanFilterMoreMeasurements
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

        process_noise = 1;
    end

    methods
        % Constructor
        function obj = LinearKalmanFilterMoreMeasurements(X, P, U, dt)
            if nargin ~= 3 && (~ismatrix(U) || ~ismatrix(X) || ~ismatrix(P))
                error('Incorrect amount of arguments passed in or incorrect arg formats.');
            end

            obj.U = U;
            obj.X = X;
            obj.P = P;

            obj.R = [1.^2, 0, 0, 0, 0;
                     0, 1.^2, 0, 0, 0;
                     0, 0, 1.^2, 0, 0;
                     0, 0, 0, .2.^2, 0;
                     0, 0, 0, 0, .5.^2;];
            obj = obj.calculateInitialValues(dt);
        end

        function obj = predictState(obj)
            obj.X = (obj.F * obj.X) + (obj.G * obj.U);
        end
        
        function obj = updateState(obj, measurement)
            obj.X = obj.X + obj.K*(measurement - obj.H*obj.X);
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
        
        function obj = calculateInitialValues(obj, dt)
            obj.F =    [1, 0, 0, dt, 0, 0;
                        0, 1, 0, 0, dt, 0;
                        0, 0, 1, 0, 0, dt;
                        0, 0, 0, 1, 0,  0;
                        0, 0, 0, 0, 1,  0;
                        0, 0, 0, 0, 0,  1];

            obj.G =    [0.5*dt*dt, 0, 0;
                        0, 0.5*dt*dt, 0;
                        0, 0, 0.5*dt*dt;
                        dt, 0,        0;
                        0, dt,        0;
                        0, 0,       dt];
            obj.H =    [1, 0, 0, 0, 0, 0;
                        0, 1, 0, 0, 0, 0;
                        0, 0, 1, 0, 0, 0;
                        0, 0, 1, 0, 0, 0;
                        0, 0, 1, 0, 0, 0];
            
            obj.Q = [(dt^4)/4, 0, 0, (dt^3)/2, 0, 0;
                     0, (dt^4)/4, 0, 0, (dt^3)/2, 0;
                     0, 0, (dt^4)/4, 0, 0, (dt^3)/2;
                     (dt^3)/2, 0, dt^2, 0, 0, 0;
                     0, (dt^3)/2, 0, 0, dt^2, 0;
                     0, 0, (dt^3)/2, 0, 0, dt^2]*obj.process_noise*obj.process_noise;
            obj = predictState(obj);
            obj = covarianceExtrapolate(obj);
        end

        function obj = iterate(obj, dt, measurement, control)
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
            if (~isOutlier(obj, measurement, 200))
                obj = calculateKalmanGain(obj);
                obj = updateState(obj, measurement);
                obj = covarianceUpdate(obj);
                obj = predictState(obj);
                obj = covarianceExtrapolate(obj);
            end
        end

       function output = isOutlier(obj, measurement, threshold)

           gpsZResid = abs(measurement(3) - obj.X(3));
           baro1Resid = abs(measurement(4) - obj.X(3));
           baro2Resid = abs(measurement(5) - obj.X(3));

            % Check for outlier
            output = (gpsZResid > threshold) || (baro1Resid > threshold) || (baro2Resid > threshold);
       end
    end
end


