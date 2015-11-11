function[output] = guidance_system(input)

%% Initialization of global variables

persistent initial_values_defined

persistent waypoints
persistent previous_waypoint_index

if isempty(initial_values_defined)
    
    initial_values_defined = 1;
    
    waypoints = waypoints_collection;

    previous_waypoint_index = 1;
end

%% Initialization of local variables

model_parameters = get_model_parameters();

previous_waypoint = waypoints.get_point(previous_waypoint_index);
next_waypoint     = waypoints.get_point(previous_waypoint_index + 1);

current_position = [input(2); input(1)];                        % [E, N]

%% Check if next waypoint is reached

if (norm(next_waypoint - current_position) < model_parameters.R_ && previous_waypoint_index < (waypoints.N_points - 1))
    previous_waypoint_index = previous_waypoint_index + 1;
end

%% Compute desired course and speed

delta_waypoint = next_waypoint - previous_waypoint;
alpha          = atan2(delta_waypoint(1), delta_waypoint(2));   % Path parallell course

delta_position = current_position - previous_waypoint;
delta_position = delta_position([2 1]);                         % Swap to [N, E] instead of [E, N]

R = [cos(alpha) -sin(alpha);
     sin(alpha) cos(alpha)];

epsilon = R' * delta_position;
s = epsilon(1);
e = epsilon(2);
lookahead_distance = 400;

desired_course = alpha + atan2(-e, lookahead_distance);
desired_speed  = 10;

output = [desired_course, desired_speed, previous_waypoint_index, e];
