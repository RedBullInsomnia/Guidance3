function guidance_system(block)
% Level-2 MATLAB file S-Function for unit delay demo.
%   Copyright 1990-2009 The MathWorks, Inc.

    setup(block);
  
%endfunction

function setup(block)
  
    %% Register number of input and output ports
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 2;
  
    %% Setup functional port properties to dynamically
    %% inherited.
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
  
    block.InputPort(1).Dimensions        = 2;
    block.InputPort(1).DirectFeedthrough = false;
    
    block.OutputPort(1).Dimensions       = 1;
    block.OutputPort(2).Dimensions       = 1;
    
    %% Set block sample time to [0.1 0]
    block.SampleTimes = [-1 0];
    
    %% Set the block simStateCompliance to default (i.e., same as a built-in block)
    block.SimStateCompliance = 'DefaultSimState';
  
    %% Register methods
    block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions',    @InitConditions);  
    block.RegBlockMethod('Outputs',                 @Output);  
    block.RegBlockMethod('Update',                  @Update);
  
%endfunction

function DoPostPropSetup(block)

    %% Setup Dwork
    block.NumDworks = 2;
  
    waypoints = load('WP');
    waypoints = waypoints.WP;
    waypoints = waypoints([2 1], :);  % Swap so we use E, N for all of script
                                    % Easier when used to x,y
  
    block.Dwork(1).Name            = 'waypoints'; 
    block.Dwork(1).Dimensions      = size(waypoints);
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
  
    block.Dwork(2).Name            = 'previous_waypoint_index'; 
    block.Dwork(2).Dimensions      = 1;
    block.Dwork(2).DatatypeID      = 0;
    block.Dwork(2).Complexity      = 'Real';
    block.Dwork(2).UsedAsDiscState = true;

%endfunction

function InitConditions(block)

    %% Initialize Dwork

    waypoints = load('WP');
    waypoints = waypoints.WP;
    waypoints = waypoints([2 1], :);  % Swap so we use E, N for all of script
                                      % Easier when used to x,y

    block.Dwork(1).Data = waypoints;
    block.Dwork(2).Data = 1;
  
%endfunction

function Output(block)

    %% Output course and speed based on LOS guidance law
    
    waypoints               = block.Dwork(1).Data;
    previous_waypoint_index = block.Dwork(2).Data;
    
    previous_waypoint = waypoints(:, previous_waypoint_index);
    next_waypoint     = waypoints(:, previous_waypoint_index + 1);
    current_position  = block.InputPort(1);
    
    delta_waypoint = next_waypoint - previous_waypoint;
    alpha          = atan2(delta(1), delta(2));              % Path parallell course
    
    delta_position = current_position - next_waypoint;
    delta_position = delta_position([2 1]);
    
    R = [cos(alpha) -sin(alpha);
         sin(alpha) cos(alpha)];
     
    epsilon = R * delta_position;
    s = epsilon(1);
    e = epsilon(2);
    lookahead_distance = 100;
    
    designated_course = alpha + atan2(-e / lookahead_distance);
    designated_speed  = 2;
    
    block.OutputPort(1).Data = designated_course;
    block.OutputPort(2).Data = designated_speed;
  
%endfunction

function Update(block)

    %% Update waypoint states

    model_parameters = get_model_parameters();

    waypoints               = block.Dwork(1).Data;
    previous_waypoint_index = block.Dwork(2).Data;
    
    previous_waypoint = waypoints(:, previous_waypoint_index);
    next_waypoint     = waypoints(:, previous_waypoint_index + 1);
    current_position  = block.InputPort(1);
    
    if (norm(next_waypoint - current_position) < model_parameters.R_)
        block.Dwork(2).Data = previous_waypoint_index + 1;
    end
  
%endfunction

