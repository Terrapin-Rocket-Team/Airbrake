function [thetaMax] = motorForceAndDragForce2MaxFlap(force_motor, force_drag)
% This function calculates the maximum flap deployment angle for a given motor 
% force and drag force in the 2024/25 airbrake design. This is used to determine 
% the maximum angle at which the flap can deploy to balance these forces.
%
% Refer to Force_Geometry.png or
% https://docs.google.com/drawings/d/1QFHGP2JmfYvRHVw_ijJOn-UjHsinjqd6PMnqW1MHL7Y/edit
% for detailed math on the definition of the lengths and x2
%
% Inputs:
% - force_motor: Force generated by the motor (in any unit of force)
% - force_drag: Drag force acting on one flap (same unit as force_motor)
%
% Outputs:
% - thetaMax: Maximum flap deployment angle (in degrees)

% Geometric Constants in inches
length_flap_in = 4.5; % length of the flap
length_pushrod_hinge_in = 2; % distance from pushrod connection to hinge
length_clevis_in = 0.61; % length of clevis

% Convert geometric constants to metric units (meters)
in_to_m = 0.0254; % conversion factor from inches to meters
length_flap_m = length_flap_in * in_to_m;
length_pushrod_hinge_m = length_pushrod_hinge_in * in_to_m;
length_clevis_m = length_clevis_in * in_to_m;

% Calculations using inch values
x2 = force_motor * length_pushrod_hinge_m / force_drag;
thetaMax = asind(x2/length_flap_m);

% thetaMax = min(thetaMax, 90);
% thetaMax = max(thetaMax, 0);

end