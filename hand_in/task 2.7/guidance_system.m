function[output] = guidance_system(input)

%% Initialization of global variables

persistent initial_values_defined

persistent waypoints
persistent previous_waypoint_index
persistent settings

if isempty(initial_values_defined)
    
    initial_values_defined = 1;
    
    waypoints = waypoints_collection;
    previous_waypoint_index = 1;
    
    settings  = get_settings();
end

%% Initialization of local variables
    
current_position = [input(1); input(2)];
track            = input(3);
t                = input(4);

%% Compute output from specified guidance system

if (track == 1)
    [desired_course, desired_speed, previous_waypoint_index, e] = guidance_system_tracking(current_position, settings, waypoints, previous_waypoint_index, t);
else 
    [desired_course, desired_speed, previous_waypoint_index, e] = guidance_system_path_following(current_position, settings, waypoints, previous_waypoint_index);
end

output = [desired_course, desired_speed, previous_waypoint_index, e];

end
