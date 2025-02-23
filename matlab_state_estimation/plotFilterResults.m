function plotFilterResults(dataType, data, output, P_output)
     
% Plot Z position vs time (Actual, Measured, Output)
figure;
if dataType ~= DataType.Flight
    plot(data.t, data.r_z, 'r-', 'DisplayName', 'Actual Z Position');
end
hold on;
plot(data.t, data.r_meas1_z, 'c.', 'DisplayName', 'Measured Z Position GPS');
plot(data.t, data.r_meas2_z, 'g.', 'DisplayName', 'Measured Z Position Baro1');
plot(data.t, data.r_meas3_z, 'y.', 'DisplayName', 'Measured Z Position Baro2');
plot(data.t, output.r_output_z, 'b', 'DisplayName', 'Output Z Position');
xlabel('Time (s)', 'FontSize', 16);
ylabel('Z Position (m)', 'FontSize', 16);
title('Z Position vs Time', 'FontSize', 18);
legend('show', 'FontSize', 14);
grid on;
hold off;

% Plot x, y, z positions vs time
figure;
subplot(3,1,1);
if dataType ~= DataType.Flight
plot(data.t, data.r_x, 'r', 'DisplayName', 'Actual x');
end
hold on;
plot(data.t, data.r_meas_x, 'g.', 'DisplayName', 'Measured X Position');
plot(data.t, output.r_output_x, 'b', 'DisplayName', 'Output x');
xlabel('Time (s)');
ylabel('x Position (m)');
title('x Position vs Time');
legend show;
grid on;

subplot(3,1,2);
if dataType ~= DataType.Flight
    plot(data.t, data.r_y, 'r', 'DisplayName', 'Actual y');
end
hold on;
plot(data.t, data.r_meas_y, 'g.', 'DisplayName', 'Measured Y Position');
plot(data.t, output.r_output_y, 'b', 'DisplayName', 'Output y');
xlabel('Time (s)');
ylabel('y Position (m)');
title('y Position vs Time');
legend show;
grid on;

subplot(3,1,3);
if dataType ~= DataType.Flight
    plot(data.t, data.r_z, 'r', 'DisplayName', 'Actual z');
end
hold on;
plot(data.t, data.r_meas1_z, 'c.', 'DisplayName', 'Measured Z Position GPS');
plot(data.t, data.r_meas2_z, 'g.', 'DisplayName', 'Measured Z Position Baro1');
plot(data.t, data.r_meas3_z, 'y.', 'DisplayName', 'Measured Z Position Baro2');
plot(data.t, output.r_output_z, 'b', 'DisplayName', 'Output z');
xlabel('Time (s)');
ylabel('z Position (m)');
title('z Position vs Time');
legend show;
grid on;
hold off;

% Plot z velocity vs time
figure;
if dataType ~= DataType.Flight
    plot(data.t, data.v_z, 'r', 'DisplayName', 'Actual z Velocity');
end
hold on;
plot(data.t, output.v_output_z, 'b', 'DisplayName', 'Output z Velocity');
xlabel('Time (s)', 'FontSize', 16);
ylabel('z Velocity (m/s)', 'FontSize', 16);
title('z Velocity vs Time', 'FontSize', 18);
legend('show', 'FontSize', 14);
grid on;
hold off;

% Plot x, y, z velocities vs time
figure;
subplot(3,1,1);
if dataType ~= DataType.Flight
    plot(data.t, data.v_x, 'r', 'DisplayName', 'Actual x Velocity');
end
hold on;
plot(data.t, output.v_output_x, 'b', 'DisplayName', 'Output x Velocity');
xlabel('Time (s)');
ylabel('x Velocity (m/s)');
title('x Velocity vs Time');
legend show;
grid on;

subplot(3,1,2);
if dataType ~= DataType.Flight
    plot(data.t, data.v_y, 'r', 'DisplayName', 'Actual y Velocity');
end
hold on;
plot(data.t, output.v_output_y, 'b', 'DisplayName', 'Output y Velocity');
xlabel('Time (s)');
ylabel('y Velocity (m/s)');
title('y Velocity vs Time');
legend show;
grid on;

subplot(3,1,3);
if dataType ~= DataType.Flight
    plot(data.t, data.v_z, 'r', 'DisplayName', 'Actual z Velocity');
end
hold on;
plot(data.t, output.v_output_z, 'b', 'DisplayName', 'Output z Velocity');
xlabel('Time (s)');
ylabel('z Velocity (m/s)');
title('z Velocity vs Time');
legend show;
grid on;
hold off;

% Plot Measurement Residuals
figure;
subplot(3,1,1);
plot(data.t, output.r_output_x - data.r_meas_x, 'r', 'DisplayName', 'Pos X Meas Residual');
xlabel('Time (s)');
ylabel('x (m)');
title('x Meas Resid vs Time');
legend show;
grid on;

subplot(3,1,2);
plot(data.t, output.r_output_y - data.r_meas_y, 'r', 'DisplayName', 'Pos Y Meas Residual');
xlabel('Time (s)');
ylabel('y (m)');
title('y Meas Resid vs Time');
legend show;
grid on;

subplot(3,1,3);
hold on;
plot(data.t, output.r_output_z - data.r_meas1_z, 'r', 'DisplayName', 'Pos Z Meas Residual GPS');
plot(data.t, output.r_output_z - data.r_meas2_z, 'b', 'DisplayName', 'Pos Z Meas Residual Baro1');
plot(data.t, output.r_output_z - data.r_meas3_z, 'g', 'DisplayName', 'Pos Z Meas Residual Baro2');
xlabel('Time (s)');
ylabel('z (m)');
title('z Meas Resid vs Time');
legend show;
grid on;

% Plot State Error
if dataType ~= DataType.Flight
    figure;
    subplot(2,3,1);
    plot(data.t, output.r_output_x - data.r_x, 'r', 'DisplayName', 'Pos X State Error');
    xlabel('Time (s)');
    ylabel('x (m)');
    title('x State Error vs Time');
    legend show;
    grid on;

    subplot(2,3,2);
    plot(data.t, output.r_output_y - data.r_y, 'r', 'DisplayName', 'Pos Y State Error');
    xlabel('Time (s)');
    ylabel('y (m)');
    title('y State Error vs Time');
    legend show;
    grid on;

    subplot(2,3,3);
    plot(data.t, output.r_output_z - data.r_z, 'r', 'DisplayName', 'Pos Z State Error');
    xlabel('Time (s)');
    ylabel( 'z (m)');
    title('z State Error vs Time');
    legend show;
    grid on;

    subplot(2,3,4);
    plot(data.t, output.v_output_x - data.v_x, 'r', 'DisplayName', 'Velo X State Error');
    xlabel('Time (s)');
    ylabel('x (m/s)');
    title('x Velo State Error vs Time');
    legend show;
    grid on;

    subplot(2,3,5);
    plot(data.t, output.v_output_y - data.v_y, 'r', 'DisplayName', 'Velo Y State Error');
    xlabel('Time (s)');
    ylabel('y (m/s)');
    title('y Velo State Error vs Time');
    legend show;
    grid on;
    
    subplot(2,3,6);
    plot(data.t, output.v_output_z - data.v_z, 'r', 'DisplayName', 'Velo Z State Error');
    xlabel('Time (s)');
    ylabel('z (m/s)');
    title('z Velo State Error vs Time');
    legend show;
    grid on;
end

if dataType ~= DataType.Flight
    % Calculated NEES
    NEES = [];
    for i = 1:height(output)
        X_true = [data.r_x(i); data.r_y(i); data.r_z(i); data.v_x(i); data.v_y(i); data.v_z(i)];
        X_est = [output.r_output_x(i); output.r_output_y(i); output.r_output_z(i); 
            output.v_output_x(i); output.v_output_y(i); output.v_output_z(i)];
        NEES = [NEES; (X_true - X_est)' * P_output(:, :, i) * (X_true - X_est) / 6];
    end

    % Plot NEES
    figure;
    plot(data.t, NEES);
    xlabel('Time (s)');
    ylabel('NEES');
    ylim([0, 10])
    title('Normalized Estimation Error Squared (NEES)');
    legend show;
    grid on;
end

if dataType ~= DataType.Flight
    % Calculated NEES
    fprintf('Max Altitude Diff: %.2f [m]\n', max(data.r_z) - max(output.r_output_z));
end

end