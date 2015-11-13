function[output] = guidance_system(input)

%% Initialization of global variables

persistent initial_values_defined

persistent waypoints
persistent previous_waypoint_index
persistent model_parameters

if isempty(initial_values_defined)
    
    initial_values_defined = 1;
    
    waypoints = waypoints_collection;

    previous_waypoint_index = 1;
    
    model_parameters  = get_model_parameters();
end

%% Initialization of local variables
    
current_position = [input(1); input(2)];
track            = input(3);
t                = input(4);

%% Compute output from specified guidance system

if (track == 1)
    output = guidance_system_tracking(current_position, model_parameters, waypoints, previous_waypoint_index, t);
else 
    output = guidance_system_path_following(current_position, model_parameters, waypoints, previous_waypoint_index);
end

end
