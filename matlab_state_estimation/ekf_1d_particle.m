g = 9.81;
x0 = 0;
v0 = 50;
t_max = 10;
dt = 0.01;
t = 0:dt:t_max;
num_steps = length(t);


q_a = 0.2^2;
q_b = 0.1;

true_x = x0 + v0 * t - 0.5 * g * t.^2;
true_xdot = v0 - g * t;

sigma_x = 2;
noisy_x = true_x + sigma_x * randn(size(true_x));

z = noisy_x;

x = [x0; v0; 1e-4];
P = eye(3);

H = [1, 0, 0];

Q = [ q_a * dt^4 / 4,   q_a * dt^3 / 2,   0;
      q_a * dt^3 / 2,   q_a * dt^2,       0;
      0,                0,                q_b * dt ];

R = 1000;

x_estimates = zeros(3, num_steps);
x_estimates(:,1) = x';

for k = 2:num_steps
    x_prev = x_estimates(:, k-1);
    x = x_prev(1);
    x_dot = x_prev(2);
    beta = x_prev(3);

    rho_prev = get_density(x);

    x_prop = x + dt * x_dot + 0.5 * dt^2 * (-g - 0.5*rho_prev*beta*(x_dot)^2);  
    x_dot_prop = x_dot + dt * (-g - 0.5*rho_prev*beta*(x_dot)^2);
    beta_prop = beta;

    x_hat_pred = [x_prop; x_dot_prop; beta_prop];

    rho = get_density(x_prop);

    F = [1, 1 - 0.5*rho*dt^2*x_hat_pred(2)*x_hat_pred(3), -0.25*rho*dt^2*(x_hat_pred(2))^2;
           0, 1 - rho*dt*x_hat_pred(2)*x_hat_pred(3) , -0.5*rho*dt*(x_hat_pred(2))^2;
           0, 0, 1];

    P_pred = F * P * F' + Q;

    y = z(k) - H * x_hat_pred;
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;

    x = x_hat_pred + K * y;
    P = (eye(3) - K * H) * P_pred;

    x_estimates(:, k) = x;
end



figure;
subplot(3, 1, 1);
plot(t, true_x, 'b-', t, noisy_x, 'r');
hold on
plot(t, x_estimates(1,:))
legend('True Position', 'Noisy Position', 'Estimated Position');
xlabel('Time (s)');
ylabel('Position (m)');
title('Particle Position');

subplot(3, 1, 2);
plot(t, true_xdot, 'b-');
hold on
plot(t, x_estimates(2,:))
legend('True Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Particle Velocity');

subplot(3, 1, 3);
plot(t, x_estimates(3, :), 'b', 'DisplayName', 'Estimated Ballistic Coefficient');
title('Ballistic Coefficient');
xlabel('Time [s]');
ylabel('Ballistic Coefficient');
legend();

function density=get_density(h)
    p0=101325; %Pa
    T0=288.15; %K
    L=.0065; %Temp lapse rate
    R=8.31446; %Ideal gas constant
    M=.0289652; %molar mass of air

    density = p0*M/R/T0*(1-L*h/T0).^((9.8*M/R/L)-1);
end
