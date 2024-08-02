function football_with_magnus()
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

    % Solve ODE without drag but with Magnus effect
    [t_nodrag, y_nodrag] = ode45(@(t, y) equations_with_magnus(t, y, g, rho, 0, A, mass, omega, d, Cl), tspan, y0);

    % Solve ODE with drag and Magnus effect
    [t_drag, y_drag] = ode45(@(t, y) equations_with_magnus(t, y, g, rho, Cd, A, mass, omega, d, Cl), tspan, y0);

    % Extract results
    x_nodrag_traj = y_nodrag(:, 1); % Horizontal positions without drag
    y_nodrag_traj = y_nodrag(:, 2); % Vertical positions without drag
    x_drag_traj = y_drag(:, 1); % Horizontal positions with drag
    y_drag_traj = y_drag(:, 2); % Vertical positions with drag

    % Find horizontal distances
    x_nodrag = max(x_nodrag_traj); % Maximum horizontal distance without drag
    x_withdrag = max(x_drag_traj); % Maximum horizontal distance with drag

    % Display results
    fprintf('Horizontal distance without drag but with Magnus effect: %.2f ft\n', x_nodrag);
    fprintf('Horizontal distance with drag and Magnus effect: %.2f ft\n', x_withdrag);

    % Plot results
    figure;
    hold on;
    plot(x_nodrag_traj, y_nodrag_traj, 'b', 'DisplayName', 'Without Drag but with Magnus Effect');
    plot(x_drag_traj, y_drag_traj, 'r', 'DisplayName', 'With Drag and Magnus Effect');
    xlabel('Horizontal Distance (ft)');
    ylabel('Vertical Distance (ft)');
    title('Trajectory of the Football');
    legend show;
    grid on;

    % Animate the ball trajectories
    hBall1 = plot(x_nodrag_traj(1), y_nodrag_traj(1), 'bo', 'MarkerFaceColor', 'b');
    hBall2 = plot(x_drag_traj(1), y_drag_traj(1), 'ro', 'MarkerFaceColor', 'r');
    for k = 1:length(t_nodrag)
        set(hBall1, 'XData', x_nodrag_traj(k), 'YData', y_nodrag_traj(k));
        set(hBall2, 'XData', x_drag_traj(k), 'YData', y_drag_traj(k));
        pause(0.1); % Pause to create animation effect
    end
    hold off;
end

function dydt = equations_with_magnus(~, y, g, rho, Cd, A, mass, omega, d, Cl)
    vx = y(3);
    vy = y(4);
    v = sqrt(vx^2 + vy^2);

    % Drag force
    Fd = 0.5 * rho * v^2 * Cd * A;

    % Magnus force (perpendicular to velocity)
    Fm = Cl * rho * v^2 * A * d / (2 * mass);

    % Decompose forces
    Fdx = Fd * vx / v; % Drag force component in the x-direction
    Fdy = Fd * vy / v; % Drag force component in the y-direction
    Fmx = Fm * vy / v; % Magnus force component in the x-direction
    Fmy = -Fm * vx / v; % Magnus force component in the y-direction (upward for counterclockwise rotation)

    % Acceleration components
    ax = -Fdx / mass + Fmx; % Horizontal acceleration
    ay = -g - Fdy / mass + Fmy; % Vertical acceleration

    dydt = [vx; vy; ax; ay];
end
