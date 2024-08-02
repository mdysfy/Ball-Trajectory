function combined_football_trajectory()
    % Parameters
    g = 32.2; % Acceleration due to gravity in ft/s^2
    v0 = 80; % Initial velocity in ft/s
    theta = 45; % Launch angle in degrees
    rho = 0.00238; % Air density in slugs/ft^3
    Cd = 0.5; % Drag coefficient
    d = 6 / 12; % Diameter of the football in ft
    A = pi * (d/2)^2; % Cross-sectional area of the football in ft^2
    mass = 0.91 / 32.2; % Mass of the football in slugs (1 lbf = 1/32.2 slugs)
    omega = 300; % Rotation rate in rad/s
    Cl = 0.2; % Lift coefficient for the Magnus effect

    % Initial velocity components
    v0x = v0 * cosd(theta); % Horizontal component of initial velocity
    v0y = v0 * sind(theta); % Vertical component of initial velocity

    % Time span for ODE solver
    tspan = [0, 2 * (2 * v0y / g)]; % Time span from 0 to twice the time of flight

    % Initial state [x, y, vx, vy]
    y0 = [0; 0; v0x; v0y]; % Initial conditions: starting at origin with initial velocities

    % Solve ODE without drag and without Magnus effect
    [t_nodrag_nomagnus, y_nodrag_nomagnus] = ode45(@(t, y) equations_nodrag(t, y, g), tspan, y0);

    % Solve ODE with drag but without Magnus effect
    [t_drag_nomagnus, y_drag_nomagnus] = ode45(@(t, y) equations_with_drag(t, y, g, rho, Cd, A, mass), tspan, y0);

    % Solve ODE without drag but with Magnus effect
    [t_nodrag_magnus, y_nodrag_magnus] = ode45(@(t, y) equations_with_magnus(t, y, g, rho, 0, A, mass, omega, d, Cl), tspan, y0);

    % Solve ODE with drag and with Magnus effect
    [t_drag_magnus, y_drag_magnus] = ode45(@(t, y) equations_with_magnus(t, y, g, rho, Cd, A, mass, omega, d, Cl), tspan, y0);

    % Extract results
    x_nodrag_nomagnus = y_nodrag_nomagnus(:, 1); % Horizontal positions without drag and without Magnus effect
    y_nodrag_nomagnus = y_nodrag_nomagnus(:, 2); % Vertical positions without drag and without Magnus effect
    x_drag_nomagnus = y_drag_nomagnus(:, 1); % Horizontal positions with drag but without Magnus effect
    y_drag_nomagnus = y_drag_nomagnus(:, 2); % Vertical positions with drag but without Magnus effect
    x_nodrag_magnus = y_nodrag_magnus(:, 1); % Horizontal positions without drag but with Magnus effect
    y_nodrag_magnus = y_nodrag_magnus(:, 2); % Vertical positions without drag but with Magnus effect
    x_drag_magnus = y_drag_magnus(:, 1); % Horizontal positions with drag and with Magnus effect
    y_drag_magnus = y_drag_magnus(:, 2); % Vertical positions with drag and with Magnus effect

    % Find horizontal distances
    x_nodrag_nomagnus_max = max(x_nodrag_nomagnus); % Maximum horizontal distance without drag and without Magnus effect
    x_drag_nomagnus_max = max(x_drag_nomagnus); % Maximum horizontal distance with drag but without Magnus effect
    x_nodrag_magnus_max = max(x_nodrag_magnus); % Maximum horizontal distance without drag but with Magnus effect
    x_drag_magnus_max = max(x_drag_magnus); % Maximum horizontal distance with drag and with Magnus effect

    % Display results
    fprintf('Horizontal distance without drag and without Magnus effect: %.2f ft\n', x_nodrag_nomagnus_max);
    fprintf('Horizontal distance with drag but without Magnus effect: %.2f ft\n', x_drag_nomagnus_max);
    fprintf('Horizontal distance without drag but with Magnus effect: %.2f ft\n', x_nodrag_magnus_max);
    fprintf('Horizontal distance with drag and with Magnus effect: %.2f ft\n', x_drag_magnus_max);

    % Plot results
    figure;
    hold on;
    plot(x_nodrag_nomagnus, y_nodrag_nomagnus, 'b-', 'LineWidth', 2, 'DisplayName', 'No Drag, No Magnus');
    plot(x_drag_nomagnus, y_drag_nomagnus, 'r-', 'LineWidth', 2, 'DisplayName', 'Drag, No Magnus');
    plot(x_nodrag_magnus, y_nodrag_magnus, 'g-', 'LineWidth', 2, 'DisplayName', 'No Drag, Magnus');
    plot(x_drag_magnus, y_drag_magnus, 'k-', 'LineWidth', 2, 'DisplayName', 'Drag, Magnus');
    xlabel('Horizontal Distance (ft)');
    ylabel('Vertical Distance (ft)');
    title('Trajectory of the Football in Different Conditions');
    legend('show');
    grid on;

    % Animate the ball trajectories
    hBall1 = plot(x_nodrag_nomagnus(1), y_nodrag_nomagnus(1), 'bo', 'MarkerFaceColor', 'b');
    hBall2 = plot(x_drag_nomagnus(1), y_drag_nomagnus(1), 'ro', 'MarkerFaceColor', 'r');
    hBall3 = plot(x_nodrag_magnus(1), y_nodrag_magnus(1), 'go', 'MarkerFaceColor', 'g');
    hBall4 = plot(x_drag_magnus(1), y_drag_magnus(1), 'ko', 'MarkerFaceColor', 'k');
    maxLength = max([length(t_nodrag_nomagnus), length(t_drag_nomagnus), length(t_nodrag_magnus), length(t_drag_magnus)]);
    for k = 1:maxLength
        if k <= length(t_nodrag_nomagnus)
            set(hBall1, 'XData', x_nodrag_nomagnus(k), 'YData', y_nodrag_nomagnus(k));
        end
        if k <= length(t_drag_nomagnus)
            set(hBall2, 'XData', x_drag_nomagnus(k), 'YData', y_drag_nomagnus(k));
        end
        if k <= length(t_nodrag_magnus)
            set(hBall3, 'XData', x_nodrag_magnus(k), 'YData', y_nodrag_magnus(k));
        end
        if k <= length(t_drag_magnus)
            set(hBall4, 'XData', x_drag_magnus(k), 'YData', y_drag_magnus(k));
        end
        pause(0.1); % Pause to create animation effect
    end
    hold off;
end

function dydt = equations_nodrag(~, y, g)
    % No drag, no Magnus effect
    vx = y(3); % Horizontal velocity
    vy = y(4); % Vertical velocity

    % Calculate accelerations
    ax = 0; % No horizontal acceleration
    ay = -g; % Vertical acceleration due to gravity

    % Return the derivatives
    dydt = [vx; vy; ax; ay];
end

function dydt = equations_with_drag(~, y, g, rho, Cd, A, mass)
    % With drag, no Magnus effect
    vx = y(3); % Horizontal velocity
    vy = y(4); % Vertical velocity
    v = sqrt(vx^2 + vy^2); % Speed

    % Calculate drag force
    Fd = 0.5 * rho * v^2 * Cd * A;

    % Decompose drag force into x and y components
    Fdx = Fd * vx / v;
    Fdy = Fd * vy / v;

    % Calculate accelerations
    ax = -Fdx / mass; % Horizontal acceleration due to drag
    ay = -g - Fdy / mass; % Vertical acceleration due to gravity and drag

    % Return the derivatives
    dydt = [vx; vy; ax; ay];
end

function dydt = equations_with_magnus(~, y, g, rho, Cd, A, mass, omega, d, Cl)
    % With or without drag, with Magnus effect
    vx = y(3); % Horizontal velocity
    vy = y(4); % Vertical velocity
    v = sqrt(vx^2 + vy^2); % Speed

    % Calculate drag force
    Fd = 0.5 * rho * v^2 * Cd * A;

    % Calculate Magnus force (perpendicular to velocity)
    Fm = Cl * rho * v^2 * A * d / (2 * mass);

    % Decompose forces into x and y components
    Fdx = Fd * vx / v; % Drag force component in the x-direction
    Fdy = Fd * vy / v; % Drag force component in the y-direction
    Fmx = Fm * vy / v; % Magnus force component in the x-direction
    Fmy = -Fm * vx / v; % Magnus force component in the y-direction (upward for counterclockwise rotation)

    % Calculate accelerations
    ax = -Fdx / mass + Fmx; % Horizontal acceleration
    ay = -g - Fdy / mass + Fmy; % Vertical acceleration

    % Return the derivatives
    dydt = [vx; vy; ax; ay];
end
