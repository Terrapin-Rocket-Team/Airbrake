classdef rocket

    properties
        totalImpulse % [Ns]
        wetMass % [kg]
        dryMass % [kg]
        burnTime % constant accel approx in s
        dragCoef % dimensionaless
        crossSectionalArea % in m^2
        mdot % [kg/s]
    end
    
    methods
        % Constructor method
        function obj = rocket(totalImpulse, wetMass, dryMass, burnTime, dragCoef, crossSectionalArea)
            if nargin > 0
                obj.totalImpulse = totalImpulse;
                obj.wetMass = wetMass;
                obj.dryMass = dryMass;
                obj.burnTime = burnTime;
                obj.dragCoef = dragCoef;
                obj.crossSectionalArea = crossSectionalArea;
                obj.mdot = (wetMass-dryMass)/burnTime;
            end
        end
    end
end