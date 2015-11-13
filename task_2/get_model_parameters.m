function[model_parameters] = get_model_parameters()

model_parameters.R_ = 800;                              % Turning radius
model_parameters.L  = 300;                              % Ship length

%% Path tracking

model_parameters.s_turning_tolerance = 800;             % Along-track tolerance for
                                                        % switching active line segment
%% Target tracking

model_parameters.U_target                    = 3;
model_parameters.approach_speed              = 4;
model_parameters.tracking_distance_reference = 1000;