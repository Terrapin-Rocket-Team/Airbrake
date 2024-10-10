function [force_motor, force_pushrod] = flap2motorforce(theta, force_drag)
% This function is used to determine the force on the pushrod and motor in
% the 2024/25 airbrake design.
%
% See Force_Geometery.png or
% https://docs.google.com/drawings/d/1QFHGP2JmfYvRHVw_ijJOn-UjHsinjqd6PMnqW1MHL7Y/edit
% for the math
%
% Inputs:
% - theta: Angle in degrees of the flap opening. 0 is closed. 90 is fully
% deployed
% - force_drag: Drag force one 1 of flaps. Can be any units of force. The
% output will be in the same units
%
% Outputs:
% - force_motor: Force that the motor must take. This is done for 4 flaps.
% - force_pushrod: Internal force in compression on the pushrod. This is
% for one pushrod and in the same units of the input force. Straight rod
% assumption

% Geometric Constants
length_flap = 4.5; % inches
length_pushrod = 4.3; % inches
length_central_pushrod_connection_to_hinge = 2; % inches
length_clevis = .61; % inches

force_perpendicular = force_drag*sind(theta);

psi = deg2rad(90-theta) + atan2(2 * length_clevis, length_flap); % in rad
phi = acos((length_central_pushrod_connection_to_hinge + sqrt((length_flap/2)*(length_flap/2) + length_clevis*length_clevis)*cos(psi))/ length_pushrod); % in rad

force_pushrod = -force_perpendicular / sin(phi - psi);
force_motor = 4 * force_pushrod * sin(phi);

end