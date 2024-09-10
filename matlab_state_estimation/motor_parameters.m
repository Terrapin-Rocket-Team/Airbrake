
function [force, speed] = motor_parameters(motor_torque, motor_speed, gear_ratio, screw_lead, motor_voltage, motor_amps)

% Input parameters
% motor_torque: N-m
% motor_speed: rpm
% gear_ratio: 1:X as the gear box ratio
% screw_lead: in mm/rev
% motor_voltage: V rated voltage
% motor_amps: A rated amps

ARM_HORIZONTAL = 1.5; % in inches
ARM_HORIZONTAL_SI = ARM_HORIZONTAL * 0.0254; % in meters
LEAD_SCREW_EFFICIENCY = .5; % 0-1 (.2-.8 is general range) (.85-.95 for ball screws)
BATTERY_VOLTAGE = 3.7; % V
BATTERY_CAPACITY = 200; % mAh
BATTERY_CAPACITY_SI = BATTERY_CAPACITY / 1000;

motor_torque = motor_torque * gear_ratio;
motor_speed = motor_speed / gear_ratio;

% Make all the input parameter nice units
motor_speed_si = motor_speed * 2 * pi/60; % rad/s
screw_lead_si = screw_lead / 1000; % m/rev
motor_wattage = motor_torque*motor_speed_si; % W
fprintf('Motor Wattage: %d\n', motor_wattage);

% Determine number of batteries needed (assume the batteries can output the
% necessary voltage)
voltage_batteries = motor_voltage/BATTERY_VOLTAGE;
number_of_batteries = ceil(voltage_batteries);
fprintf('Number of Batteries: %d\n', number_of_batteries);
battery_life = BATTERY_CAPACITY_SI/motor_amps;
fprintf('Battery Life (hours): %d\n', battery_life);

speed = (ARM_HORIZONTAL_SI*2/screw_lead_si)/motor_speed_si;
force = motor_torque * 2 * pi * LEAD_SCREW_EFFICIENCY / screw_lead_si;

end




