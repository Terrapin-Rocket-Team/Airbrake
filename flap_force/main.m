% This script uses the function flap2motorforce to plot the motor and
% pushrod force for all ranges of theta and an input drag force F

% Define the range of theta (in degrees)
theta_range = 0:.1:90; % angles from 21 to 90 degrees

% Initialize arrays to store forces
force_motor = zeros(size(theta_range));
force_pushrod = zeros(size(theta_range));

force_drag = 200;

% Loop over the theta range and calculate forces
for i = 1:length(theta_range)
    theta = theta_range(i);
    [force_motor(i), force_pushrod(i)] = flap2motorforce(theta, force_drag);
end

% Plotting the results
figure;
plot(theta_range, force_motor, 'r-', 'LineWidth', 2, 'DisplayName', 'Motor Force');
hold on;
plot(theta_range, force_pushrod, 'b-', 'LineWidth', 2, 'DisplayName', 'Pushrod Force');
xlabel('Flap Angle (degrees)', 'FontSize', 14);
ylabel('Force Multiplier', 'FontSize', 14);
title('Motor and Pushrod Forces vs Flap Angle', 'FontSize', 16);
legend('show', 'FontSize', 12, 'Location', 'northwest');
grid on;
hold off;