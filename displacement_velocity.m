% Parameters
M_c = 274;    % Sprung mass (kg)
M_t = 41.1;   % Unsprung mass (kg)
M_s = 110;    % Driver mass (kg)
K_c = 17344.2;% Suspension stiffness (N/m)
K_t = 178922; % Tire stiffness (N/m)
K_s = 8000;   % Driver-seat stiffness (N/m)
C_c = 1644;   % Suspension damping coefficient (N.s/m)
C_t = 500;    % Tire damping coefficient (N.s/m)
C_s = 3000;   % Damping coefficient between driver and seat (N.s/m)

% Time vector
t = 0:0.01:10; % Extend simulation time to 10 seconds

% System matrices 
A = [0, 1, 0, 0, 0, 0;...
    -(K_c + K_t)/M_c, -C_c/M_c, K_t/M_c, C_t/M_c, 0, 0;...
    0, 0, 0, 1, 0, 0;...
    K_t/M_t, C_t/M_t, -(K_t + K_s)/M_t, -(C_t + C_s)/M_t, K_s/M_t, C_s/M_t;...
    0, 0, 0, 0, 0, 1;...
    0, 0, K_s/M_s, C_s/M_s, -K_s/M_s, -C_s/M_s];

B = [0; 1/M_c; 0; 0; 0; 0];

% Input - road profile as a function of velocity
desired_velocities = [5.55556, 8.333333, 11.1111, 13.88889]; % Different desired velocities to test

figure;

for i = 1:length(desired_velocities)
    v_desired_m_per_s = desired_velocities(i);

    % Convert velocity from m/s to km/h
    v_desired_km_per_h = v_desired_m_per_s * 3.6;

    % Input - road profile as a function of velocity
    u = 0.01 * sin(2 * pi * 1 * t) + 0.02 * (v_desired_m_per_s - zeros(size(t)));

    % Simulation
    sys = ss(A, B, eye(6), 0);
    [y, ~, x] = lsim(sys, u, t);

    % Compute acceleration
    acc_driver = gradient(x(:, 5), t);
    acc_rms = rms(acc_driver);

    % Plotting results
    subplot(length(desired_velocities), 2, 2*i-1);

    plot(t, y(:,1), 'LineWidth', 1.5, 'DisplayName', 'Sprung Mass');
    hold on;
    plot(t, y(:,3), '--', 'LineWidth', 1.5, 'DisplayName', 'Unsprung Mass');
    plot(t, y(:,5), '-.', 'LineWidth', 1.5, 'DisplayName', 'Driver');

    xlabel('Time (s)');
    ylabel('Displacement (m)');
    title(['Desired Velocity = ' num2str(v_desired_km_per_h) ' km/h']);
    legend('Location', 'Best');
    grid on;

    if i == length(desired_velocities)
        xlabel('Time (s)');
    end

    % Plot RMS acceleration versus velocity
    subplot(length(desired_velocities), 2, 2*i);

    plot(v_desired_km_per_h, acc_rms, 'o', 'LineWidth', 1.5, 'DisplayName', 'RMS Acceleration');
    xlabel('Velocity (km/h)');
    ylabel('RMS Acceleration (m/s^2)');
    title(['RMS Acceleration vs. Velocity']);
    grid on;
    legend('Location', 'Best');
end
