function rideComfortAndRoadHoldingAnalysis()
    % Parameters
    ms = 300;   % Sprung mass (kg)
    mu = 30;    % Unsprung mass (kg)
    g = 9.81;   % Acceleration due to gravity (m/s^2)
    kt = 20000; % Tire stiffness (N/m)
    zr = 0;     % Displacement of the rear unsprung mass (m)
    humpAmplitude = 0.02; % Amplitude of the road hump (m)
    humpFrequency = 1;    % Frequency of the road hump (Hz)

    % Range of spring constants
    springConstants = logspace(4, 6, 10); % Adjust as needed

    % Initialize arrays to store results
    roadHoldingDamping = zeros(size(springConstants));
    rideComfortDamping = zeros(size(springConstants));

    % Simulate for different spring constants
    for i = 1:length(springConstants)
        ks = springConstants(i);

        % Solve the system of differential equations
        options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
        [~, y] = ode45(@quarterCarModel, [0 10], [0; 0; 0; 0], options, ms, mu, 500, ks, g, kt, zr, humpAmplitude, humpFrequency);

        % Calculate road holding (negative of the minimum displacement of unsprung mass)
        roadHoldingDamping(i) = -min(y(:, 3));

        % Calculate ride comfort (RMS of acceleration of the sprung mass)
        acceleration_sprung = gradient(y(:, 2), y(:, 1));
        rideComfortDamping(i) = rms(acceleration_sprung);
    end

    % Plot the results
    figure;

    % Road holding vs Spring constant
    subplot(2, 1, 1);
    loglog(springConstants, roadHoldingDamping, 'o-', 'LineWidth', 2);
    title('Road Holding vs Spring Constant');
    xlabel('Spring Constant (N/m)');
    ylabel('Road Holding (m)');
    grid on;

    % Ride comfort vs Spring constant
    subplot(2, 1, 2);
    loglog(springConstants, rideComfortDamping, 's-', 'LineWidth', 2);
    title('Ride Comfort vs Spring Constant');
    xlabel('Spring Constant (N/m)');
    ylabel('Ride Comfort (m/s^2)');
    grid on;
end

function dydt = quarterCarModel(~, y, ms, mu, cs, ks, ~, kt, zr, humpAmplitude, humpFrequency)
    % System of differential equations
    dydt = zeros(4, 1);

    % Extract state variables
    zs = y(1);
    z_dot_s = y(2);
    zu = y(3);
    z_dot_u = y(4);

    % Road profile - single hump
    roadProfile = humpAmplitude * sin(2 * pi * humpFrequency * y(1));

    % Equations of motion
    dydt(1) = z_dot_s;
    dydt(2) = (1/ms) * (-500 * (z_dot_s - z_dot_u) - ks * (zs - zu) - ms * 9.81 + roadProfile);
    dydt(3) = z_dot_u;
    dydt(4) = (1/mu) * (500 * (z_dot_s - z_dot_u) + ks * (zs - zu) - kt * (zu - zr) - mu * 9.81);
end
