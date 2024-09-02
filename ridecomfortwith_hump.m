function rideComfortAnalysisWithHump()
    % Parameters
    ms = 300;   % Sprung mass (kg)
    mu = 30;    % Unsprung mass (kg)
    ks = 10000; % Spring constant between sprung and unsprung mass (N/m)
    g = 9.81;   % Acceleration due to gravity (m/s^2)
    kt = 20000; % Tire stiffness (N/m)
    zr = 0;     % Displacement of the rear unsprung mass (m)
    humpAmplitude = 0.02; % Amplitude of the road hump (m)
    humpFrequency = 1;    % Frequency of the road hump (Hz)

    % Range of damping coefficients
    dampingCoefficients = logspace(2, 4, 10); % Adjust as needed

    % Initialize arrays to store results
    roadHolding = zeros(size(dampingCoefficients));
    rideComfort = zeros(size(dampingCoefficients));

    % Simulate for different damping coefficients
    for i = 1:length(dampingCoefficients)
        cs = dampingCoefficients(i);

        % Solve the system of differential equations
        options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
        [~, y] = ode45(@quarterCarModel, [0 10], [0; 0; 0; 0], options, ms, mu, cs, ks, g, kt, zr, humpAmplitude, humpFrequency);

        % Calculate road holding (negative of the minimum displacement of unsprung mass)
        roadHolding(i) = -min(y(:, 3));

        % Calculate ride comfort (RMS of acceleration of the sprung mass)
        acceleration_sprung = gradient(y(:, 2), y(:, 1));
        rideComfort(i) = rms(acceleration_sprung);
    end

    % Plot the results
    figure;
    
    subplot(2, 1, 1);
    loglog(dampingCoefficients, roadHolding, 'o-', 'LineWidth', 2);
    title('Road Holding vs Damping Coefficient');
    xlabel('Damping Coefficient (N.s/m)');
    ylabel('Road Holding (m)');
    grid on;

    subplot(2, 1, 2);
    loglog(dampingCoefficients, rideComfort, 's-', 'LineWidth', 2);
    title('Ride Comfort vs Damping Coefficient');
    xlabel('Damping Coefficient (N.s/m)');
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
    dydt(2) = (1/ms) * (-cs * (z_dot_s - z_dot_u) - ks * (zs - zu) - ms * 9.81 + roadProfile);
    dydt(3) = z_dot_u;
    dydt(4) = (1/mu) * (cs * (z_dot_s - z_dot_u) + ks * (zs - zu) - kt * (zu - zr) - mu * 9.81);
end