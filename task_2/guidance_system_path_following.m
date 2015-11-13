function[output] = guidance_system_path_following(current_position, model_parameters, waypoints, previous_waypoint_index)
    
previous_waypoint = waypoints.get_point(previous_waypoint_index);
next_waypoint     = waypoints.get_point(previous_waypoint_index + 1);

delta_waypoint = next_waypoint - previous_waypoint;
alpha          = atan2(delta_waypoint(2), delta_waypoint(1));   % Path parallell course

delta_position = current_position - previous_waypoint;

R = [cos(alpha) -sin(alpha);
     sin(alpha) cos(alpha)];

epsilon = R' * delta_position;

s_reference = delta_waypoint;
s = norm(s_reference) - epsilon(1);                             % Along-track error

e = epsilon(2);                                                     % Cross-track error

%% Compute desired course and speed

lookahead_distance = 2 * model_parameters.L;

desired_course = alpha + atan2(-e, lookahead_distance);
desired_speed  = model_parameters.U_target + model_parameters.approach_speed * s / sqrt(s^2 + 1000^2);

desired_speed = saturate(desired_speed, 2.5, Inf);
    
%% Check if next waypoint is reached

if (s < model_parameters.s_turning_tolerance && previous_waypoint_index < (waypoints.N_points - 1))
    previous_waypoint_index = previous_waypoint_index + 1;
end
    
%%  Output

output = [desired_course, desired_speed, previous_waypoint_index, e];

end