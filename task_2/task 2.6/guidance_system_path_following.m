function [desired_course, desired_speed, previous_waypoint_index, e] = guidance_system_path_following(current_position, settings, waypoints, previous_waypoint_index)
    
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

s_reference = delta_waypoint;                   % Want to reach next waypoint
s           = norm(s_reference) - epsilon(1);   % Epsilon gives along-track from previous waypoint

e           = epsilon(2);

%% Compute desired course and speed

desired_course = alpha + atan2(-e, settings.path_following.lookahead_distance);
desired_speed  = settings.path_following.base_speed * s / sqrt(s^2 + 1000^2);

desired_speed = saturate(desired_speed, 2.5, Inf);
    
%% Check if next waypoint is reached

if (s < settings.path_following.s_tolerance && previous_waypoint_index < (waypoints.N_points - 1))
    previous_waypoint_index = previous_waypoint_index + 1;
end

end