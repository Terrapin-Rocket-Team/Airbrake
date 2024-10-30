% This script uses the function flap2motorforce to plot the motor and
% pushrod force for all ranges of theta and an input drag force F

% Define the range of theta (in degrees)
force_drag_range = 0:1:3000; % N

% Initialize arrays to store forces
angles = zeros(size(force_drag_range));

force_motor = 800;

% Loop over the theta range and calculate forces
for i = 1:length(force_drag_range)
    force_drag = force_drag_range(i);
    angles(i) = motorForceAndDragForce2MaxFlap(force_motor, force_drag);
end

% Plotting the results
figure;
plot(force_drag_range, angles, 'r-', 'LineWidth', 2);
hold on;
xlabel('Drag Force (N)', 'FontSize', 14);
ylabel('Max Deployment Angle (deg)', 'FontSize', 14);
title(['Max Deployment Angle vs Drag Force (Motor Force = ' num2str(force_motor) ' N)'], 'FontSize', 16);
ylim([0,90]);
grid on;
hold off;