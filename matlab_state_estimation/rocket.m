classdef rocket

    properties
        motorAccel % constant accel approx in m/s^2
        burnTime % constant accel approx in s
        dragCoef % dimensionaless
        crossSectionalArea % in m^2
    end
    
    methods
        % Constructor method
        function obj = rocket(motorAccel, burnTime, dragCoef, crossSectionalArea)
            if nargin > 0
                obj.motorAccel = motorAccel;
                obj.burnTime = burnTime;
                obj.dragCoef = dragCoef;
                obj.crossSectionalArea = crossSectionalArea;
            end
        end
    end
end