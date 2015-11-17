function [desired_course, desired_speed, previous_waypoint_index, e] = guidance_system_tracking(current_position, settings, waypoints, previous_waypoint_index, t)

previous_waypoint = waypoints.get_point(previous_waypoint_index);
next_waypoint     = waypoints.get_point(previous_waypoint_index + 1);

%% Compute projection of position down to LOS line

delta_waypoint = next_waypoint - previous_waypoint;
alpha          = atan2(delta_waypoint(2), delta_waypoint(1));   % Path parallell course

delta_position = current_position - previous_waypoint;

R = [cos(alpha) -sin(alpha);
     sin(alpha) cos(alpha)];

epsilon = R' * delta_position;

%% Compute along-track(s) and cross-track(e) errors

% Target begins at second waypoint

target_path_direction = delta_waypoint / norm(delta_waypoint);
target_position       = delta_waypoint + target_path_direction * settings.tracking.target_speed * t;

% Want to be tracking_distance_reference behind the target

s_reference = target_position - target_path_direction *  settings.tracking.distance_reference_s;
s           = norm(s_reference) - epsilon(1);
    
e           = epsilon(2) - settings.tracking.distance_reference_e;

%% Compute desired course and speed

desired_course = alpha + atan2(-e, settings.tracking.lookahead_distance);
desired_speed  = settings.tracking.target_speed + settings.tracking.approach_speed * s / sqrt(s^2 + 1000^2);

desired_speed = saturate(desired_speed, 2.5, Inf);

end