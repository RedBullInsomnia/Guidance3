function[settings] = get_settings()

%% Ship properties

settings.ship.R_ = 800;                         		% Turning radius
settings.ship.L  = 300;                      			% Ship length

%% Path following

settings.path_following.s_tolerance 		= 800;      % Along-track tolerance for
                                                		% switching active line segment

settings.path_following.base_speed      	= 7;
settings.path_following.lookahead_distance 	= 2 * settings.ship.L;

%% Target tracking

settings.tracking.target_speed          	= 3;
settings.tracking.approach_speed     		= 4;

settings.tracking.distance_reference_s    	= 0;
settings.tracking.distance_reference_e    	= -500;

settings.tracking.lookahead_distance    	= 2 * settings.ship.L;