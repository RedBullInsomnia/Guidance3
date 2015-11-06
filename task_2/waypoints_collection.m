classdef waypoints_collection
   
   properties
        waypoints;          % Array of waypoints, ith column is ith waypoint
                            % in [E, N] dimensions, to comply with common x,y
                        
        N_points;
        N_lines;
   end
   
   methods
       
       %% Constructor
       
       function obj = waypoints_collection()
           
            obj.waypoints = load('WP');
            obj.waypoints = obj.waypoints.WP;
            obj.waypoints = obj.waypoints([2 1], :);    % Swap so we use E, N for all of script
                                                        % Easier when used to x,y
       
            obj.N_points = size(obj.waypoints, 2);
            obj.N_lines  = obj.N_points - 1;
       end
       
       %% Utilities
       
       function point = get_point(obj, i)
          
           point = obj.waypoints(:, i);
       end
       
       %% Plot markers on waypoint positions
       
       function plot_markers(obj, color)
            
            plot(obj.waypoints(1, :), obj.waypoints(2, :), ['o' color], 'MarkerFaceColor', color);
       end
       
       %% Straight line
       
       function plot_piecewise_continuous(obj, color)
            
            plot(obj.waypoints(1, :), obj.waypoints(2, :), color);
       end
       
       %% Cubic Hermitian interpolation
       
       function plot_continuous_interpolation(obj, color)
           
            % When to be at the different waypoints(not important, just needed for pchip alogrithm)

            waypoint_timestep = 20;
            waypoint_times    = 0:waypoint_timestep:(waypoint_timestep * (obj.N_points - 1));

            time = 0:0.1:max(waypoint_times);

            path_E = pchip(waypoint_times, obj.waypoints(1, :), time);
            path_N = pchip(waypoint_times, obj.waypoints(2, :), time);

            plot(path_E, path_N, color);
       end
       
       %% Turning circles
          
       function plot_circles(obj, color)
            
            model_parameters = get_model_parameters();
           
            % Vectors from waypoint n to waypoint n + 1

            waypoint_lines = nan(2, obj.N_lines);

            for i = 1:(obj.N_points - 1)
               waypoint_lines(1, i) = obj.waypoints(1, i + 1) - obj.waypoints(1, i);
               waypoint_lines(2, i) = obj.waypoints(2, i + 1) - obj.waypoints(2, i);
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

                plot(tangent_point_1(1), tangent_point_1(2), ['x' color]);
                plot(tangent_point_2(1), tangent_point_2(2), ['x' color]);

                % Find normal vector to both tangents

                normal_1 = [-line_1(2) line_1(1)] / norm(line_1);
                normal_2 = [-line_2(2) line_2(1)] / norm(line_2);

                % Find equation for the lines of the normal vectors

                a = normal_1(2) / normal_1(1);
                b = -a * tangent_point_1(1) + tangent_point_1(2);

                c = normal_2(2) / normal_2(1);
                d = -c * tangent_point_2(1) + tangent_point_2(2);

                % Where lines cross is the center of the circle

                cross_E = (b - d) / (c - a);
                cross_N = a * cross_E + b;

                plot(cross_E, cross_N, ['+' color]);

                % Draw circle

                theta = linspace(0, 2 * pi, 1000);

                circle_E = cross_E + R_ * cos(theta);
                circle_N = cross_N + R_ * sin(theta);

                plot(circle_E, circle_N, ['--' color]);
            end
       end
   end
end