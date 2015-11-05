
clc;
clear all;

waypoints = load('WP');
waypoints = waypoints.WP;
waypoints = waypoints([2 1], :);    % Swap so we use E, N for all of script
                                    % Easier when used to x,y

waypoints_E = waypoints(1, :)';
waypoints_N = waypoints(2, :)';
N = length(waypoints_N);

% When to be at the different waypoints(not important except for generation step

waypoint_timestep = 20;
waypoint_times    = 0:waypoint_timestep:(waypoint_timestep * (N - 1));

% Vectors from waypoint n to waypoint n + 1

waypoint_lines    = nan(2, N - 1);
N_lines = length(waypoint_lines);

for i = 1:(N - 1)
   waypoint_lines(1, i) = waypoints_E(i + 1) - waypoints_E(i);
   waypoint_lines(2, i) = waypoints_N(i + 1) - waypoints_N(i);
end

%% Waypoints

figure(1);
clf();
hold on;

legend_labels = {};

plot(waypoints_E, waypoints_N, 'ko', 'MarkerFaceColor', 'k');
legend_labels = {legend_labels{:} 'Waypoints'};

%% Continuous interpolation

time = 0:0.1:max(waypoint_times);

path_E = pchip(waypoint_times, waypoints_E, time);
path_N = pchip(waypoint_times, waypoints_N, time);

plot(path_E, path_N, 'r');
legend_labels = {legend_labels{:} 'Cubic Hermite interpolation'};

%% Piecewise continuous interpolation

plot(waypoints_E, waypoints_N, 'b');
legend_labels = {legend_labels{:} 'Piecewise continuous interpolation'};

%% Circles and straight lines

for i = 1:(N_lines - 1)
    
    % Calculate angle between lines
    
    line_1 = waypoint_lines(:, i);
    line_2 = waypoint_lines(:, i + 1);
    
    dot_product = -line_1' * line_2;
    norms = norm(line_1) * norm(line_2);
    
    alpha = (1 / 2) * (acos(dot_product / norms));
    
    % Turning radius and start-to-turn distance to point
    
    R_ = 800;
    R  = R_ / tan(alpha);
    
    % Find tangent points on either line
    
    k_1 = -R / norm(line_1);
    k_2 =  R / norm(line_2);
    
    tangent_point_1 = waypoints(:, i + 1) + (line_1 * k_1);
    tangent_point_2 = waypoints(:, i + 1) + (line_2 * k_2);
    
    plot(tangent_point_1(1), tangent_point_1(2), 'bx');
    plot(tangent_point_2(1), tangent_point_2(2), 'bx');
    
    % Find normal vector to both tangents
    
    normal_1 = [-line_1(2) line_1(1)] / norm(line_1);
    normal_2 = [-line_2(2) line_2(1)] / norm(line_2);
    
    % Find equation for the lines of the normal vectors
    
    a = normal_1(2) / normal_1(1);
    b = -a * tangent_point_1(1) + tangent_point_1(2);
    
    c = normal_2(2) / normal_2(1);
    d = -c * tangent_point_2(1) + tangent_point_2(2);
    
    % Where lines cross is the center of the circle
    
    cross_E = (b - d) / (c - a);
    cross_N = a * cross_E + b;
    
    plot(cross_E, cross_N, 'b+');
    
    % Draw circle
    
    theta = linspace(0, 2 * pi, 1000);
    
    circle_E = cross_E + R_ * cos(theta);
    circle_N = cross_N + R_ * sin(theta);
    
    plot(circle_E, circle_N, 'b--');
end

legend_labels = {legend_labels{:} 'Circles'};

%% Figure properties

legend(legend_labels);

ylabel('Position, north');
xlabel('Postition, east');

axis equal;
