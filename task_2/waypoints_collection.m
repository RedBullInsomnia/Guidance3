classdef waypoints_collection
   
   properties
        waypoints;          % Array of waypoints, ith column is ith waypoint
                            % in [N, E] dimensions
                                    
        north;              % Indexes for accessing the waypoint collection in
        east;               % a more readable way
        
        N_points;
        N_lines;
   end
   
   methods
       
       %% Constructor
       
       function obj = waypoints_collection()
           
            obj.waypoints = load('WP');
            obj.waypoints = obj.waypoints.WP;
            
            obj.north = 1;
            obj.east  = 2;
       
            obj.N_points = size(obj.waypoints, 2);
            obj.N_lines  = obj.N_points - 1;
       end
       
       %% Utilities
       
       function point = get_point(obj, i)
          
           point = obj.waypoints(:, i);
       end
       
       %% Plot markers on waypoint positions
       
       function plot_markers(obj, color)
            
            plot(obj.waypoints(obj.east, :), obj.waypoints(obj.north, :), ['o' color], 'MarkerFaceColor', color);
       end
       
       %% Straight line
       
       function plot_piecewise_continuous(obj, color)
            
            plot(obj.waypoints(obj.east, :), obj.waypoints(obj.north, :), color);
       end
       
       %% Cubic Hermitian interpolation
       
       function plot_continuous_interpolation(obj, color)
           
            % When to be at the different waypoints(not important, just needed for pchip alogrithm)

            waypoint_timestep = 20;
            waypoint_times    = 0:waypoint_timestep:(waypoint_timestep * (obj.N_points - 1));

            time = 0:0.1:max(waypoint_times);

            path_north = pchip(waypoint_times, obj.waypoints(obj.north, :), time);
            path_east  = pchip(waypoint_times, obj.waypoints(obj.east, :), time);

            plot(path_east, path_north, color);
       end
       
       %% Turning circles
          
       function plot_circles(obj, color)
            
            model_parameters = get_model_parameters();
           
            % Vectors from waypoint n to waypoint n + 1

            waypoint_lines = nan(2, obj.N_lines);

            for i = 1:(obj.N_points - 1)
               waypoint_lines(obj.north, i) = obj.waypoints(obj.north, i + 1) - obj.waypoints(obj.north, i);
               waypoint_lines(obj.east, i)  = obj.waypoints(obj.east, i + 1)  - obj.waypoints(obj.east, i);
            end
            
            % Calculate and draw circles
           
            for i = 1:(obj.N_lines - 1)
            
                % Calculate angle between lines

                line_1 = waypoint_lines(:, i);
                line_2 = waypoint_lines(:, i + 1);

                dot_product = -line_1' * line_2;
                norms = norm(line_1) * norm(line_2);

                alpha = (1 / 2) * (acos(dot_product / norms));

                % Turning radius and start-to-turn distance to point

                R_ = model_parameters.R_;
                R  = R_ / tan(alpha);

                % Find tangent points on either line

                k_1 = -R / norm(line_1);
                k_2 =  R / norm(line_2);

                tangent_point_1 = obj.waypoints(:, i + 1) + (line_1 * k_1);
                tangent_point_2 = obj.waypoints(:, i + 1) + (line_2 * k_2);

                plot(tangent_point_1(obj.east), tangent_point_1(obj.north), ['x' color]);
                plot(tangent_point_2(obj.east), tangent_point_2(obj.north), ['x' color]);

                % Find unit normal vector for both tangents

                normal_1 = [-line_1(2) line_1(1)] / norm(line_1);
                normal_2 = [-line_2(2) line_2(1)] / norm(line_2);

                % Find equation for the lines of the normal vectors

                a = normal_1(obj.north) / normal_1(obj.east);
                b = -a * tangent_point_1(obj.east) + tangent_point_1(obj.north);

                c = normal_2(obj.north) / normal_2(obj.east);
                d = -c * tangent_point_2(obj.east) + tangent_point_2(obj.north);

                % Where lines cross is the center of the circle

                cricle_center_east  = (b - d) / (c - a);
                circle_center_north = a * cricle_center_east + b;

                plot(cricle_center_east, circle_center_north, ['+' color]);

                % Draw circle

                theta = linspace(0, 2 * pi, 1000);

                circle_north = circle_center_north + R_ * sin(theta);
                circle_east  = cricle_center_east  + R_ * cos(theta);

                plot(circle_east, circle_north, ['--' color]);
            end
       end
   end
end