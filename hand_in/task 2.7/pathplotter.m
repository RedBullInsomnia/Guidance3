function pathplotter(x, y,  psi, tsamp, dec, tstart, tstop, track, WP, u, v)
%PATHPLOTTER draws the path of the ship MS Fartoystyring used in TTK4190 
%Guidance and Control Assignment 3, Tasks 2.3 to 2.7, inclusive.
%
%PATHPLOTTER outputs a single xy-plot of the MS Fartoystyring's trajectory 
%and, depending upon the input, either waypoints and path or the trajectory
%of the target. The MS Fartoystyring is yellow and the target green. MS
%Fartoystyring's trajectory is blue, and the waypoints/target trajectory
%red.
%
%x      is the ship's north position (in NED). x is a vector in R^T, T is 
%          number of samples
%y      is the ship's east position (in NED). y is a vector in R^T 
%psi    is the ship's yaw angle (in NED). psi is a vector in R^T 
%tsamp  is the sampling time
%dec>=1 is how much of the data should be used in the plot. dec should 
%          be a natural number. E.g. dec=2 reduces number of data points by
%          a factor of 2. Too low value of dec makes the plot illegible
%tstart is simulation start time
%tstop  is simulation stop time
%track  is a boolean value indicating whether or not the ship is following 
%          waypoints (Tasks 2.3-2.6) (track=0) or tracking a target (Task
%          2.7) (track=1).
%
%Input must be sampled with a fixed time step.
%
%Author   : Christian Holden
%Date     : 2008-02-22
%Revisions: 2009-01-15 C. Holden.       Updated for new semester. 
%           2010-02-22 C. Holden.       Updated for new semester.
%           2010-03-02 C. Holden.       Minor bug fix
%           2015-11-06 Postmann Pat     Major clean up. Smaller bug fixes.
%
%You are free to modify the code as necessary.
%
%Bugs should be reported to the TA.

close all;
figure();
hold on;

% Just indexes for accessing WP array in a more readable way

north = 1;                  
east  = 2;

% What to plot

should_plot_speed_vectors                   = true;

should_plot_distance_to_target              = true;
should_plot_distance_to_target_element_wise = true;

should_plot_active_line_segment             = true;
should_plot_along_track_errors              = false;
should_plot_cross_track_errors              = true;

% Speed of the target

U_target = 3;

%% Plot waypoints and paths

% Angle between first two waypoints (and targets orientation)

psiTemp = atan2(WP(east, 2) - WP(east ,1), WP(north, 2) - WP(north, 1));

% Plot line where the target is moving

if (track)

    target_end_east    = WP(east, 2) + U_target * sin(psiTemp) * (tstop - tstart);
    target_end_north   = WP(north, 2) + U_target * cos(psiTemp) * (tstop - tstart);

    plot([WP(east, 1), target_end_east], [WP(north, 1), target_end_north], 'r');

% Plot lines between waypoints and markers at waypoints

else    
    [~, N_WP] = size(WP);
    
    for ii = 1:(N_WP - 1)
        plot([WP(east, ii), WP(east, ii + 1)], [WP(north, ii), WP(north, ii + 1)], 'r-x');
    end
end

% Plot MS Fartoystyrings path from simulation results

plot(y, x)

%% Plot small ship models along the paths

% L is length of the ships
% target is the geometric description of the target for drawing, used with patch function
% boat is the geometric description of the target for drawing, used with patch function

L = 300;

R = [cos(psiTemp) -sin(psiTemp);        % The orientation of the target
     sin(psiTemp) cos(psiTemp)];

target = R * [L/2 .9*L/2 .5*L/2 -L/2 -L/2 .5*L/2 .9*L/2 L/2; 
              0 10 20 20 -20 -20 -10 0];

tnow = tstart;

for now = 1:dec:length(x)
    
    %Target

    if (track)
        
        plot(WP(east, 2)  + U_target * sin(psiTemp) * (tnow - tstart) + target(2,:), WP(north, 2) + U_target * cos(psiTemp) * (tnow - tstart) + target(1, :), 'g');
        patch(WP(east, 2) + U_target * sin(psiTemp) * (tnow - tstart) + target(2,:), WP(north, 2) + U_target * cos(psiTemp) * (tnow - tstart) + target(1, :), 'g');
    end
    
    %MS Fartoystyring

    tmpR = [cos(psi(now)) -sin(psi(now));
            sin(psi(now)) cos(psi(now))];

    boat = tmpR*[L/2 .9*L/2 .5*L/2 -L/2 -L/2 .5*L/2 .9*L/2 L/2; 
                 0 10 20 20 -20 -20 -10 0];

    plot(y(now) + boat(2,:), x(now) + boat(1, :), 'y');
    patch(y(now) + boat(2,:), x(now) + boat(1, :), 'y');
    
    v_now = tmpR * [u(now) v(now)]';
    
    if (should_plot_speed_vectors)
       plot([y(now) (y(now) + 50 * v_now(2))], [x(now) (x(now) + 50 * v_now(1))], 'k', 'LineWidth', 3);
    end
    
    tnow = tnow + tsamp * dec;
end

hold off;
xlabel('East [m]');
ylabel('North [m]');
axis equal;

%% Additional information

% Tracking errors when tracking target

if (track)

    tim = tstart:tsamp:tstop;
    
    st = size(tim);
    sy = size(y);

    if(st(1) ~= sy(1))
        tim = tim';
    end
    
    dx = WP(north, 2) + U_target * cos(psiTemp) * tim - x;
    dy = WP(east, 2)  + U_target * sin(psiTemp) * tim - y;
    
    if (should_plot_distance_to_target)

        figure();
        plot(tim, sqrt(dx.^2 + dy.^2));
        xlabel('Time [s]');
        ylabel('Distance [m]');
        title('Distance to target');
    end
    
    if (should_plot_distance_to_target_element_wise)
        
        figure();
        hold on;
        plot(tim, dx, 'r');
        plot(tim, dy);
        xlabel('Time [s]');
        ylabel('Distance [m]');
        title('Distance to target');
        legend('x', 'y');
        hold off;
    end

% Along and cross track errors to the path

else
    sw   = size(WP);
    alph = zeros(max(sw) - 1, 1);
    
    % Compute 'course' of each waypoint line

    for ii = 1:length(alph)
        alph(ii) = atan2(WP(east, ii+1) - WP(east, ii), WP(north, ii+1) - WP(north, ii));
    end

    tim = tstart:tsamp:tstop;

    e = zeros(length(tim), 1);          % Cross-track error at each timestep
    s = zeros(length(tim), 1);          % Along-track error at each timestep

    tmp_e = zeros(size(alph));
    tmp_s = zeros(size(alph));

    s_line_switch_tolerance = 5;        % How small s should be when switching active line
    mind = 1;                           % Current line segment
    minds = zeros(size(tim));           % Timeseries of active line segments

    for ii = 1:length(tim)

        for jj = mind:length(tmp_e)    % Calculate errors to all remaining lines

            % First find along track distance from previous waypoint
            % Error is then the remaining part

            tmp_s(jj) = (x(ii) - WP(north, jj)) * cos(alph(jj)) + (y(ii) - WP(east, jj)) * sin(alph(jj));
            tmp_s(jj) = sqrt((WP(north, jj) - WP(north, jj + 1))^2 + (WP(east, jj) - WP(east, jj + 1))^2) - tmp_s(jj);
            
            tmp_e(jj) = -(x(ii)-WP(north, jj)) * sin(alph(jj)) + (y(ii) - WP(east, jj)) * cos(alph(jj));
        end

        % If we are close enough to the target waypoint, we switch line segments

        if(tmp_s(mind) < s_line_switch_tolerance)
            mind = mind + 1;
        end

        % If we are past the last line, do not increment

        if (mind > length(tmp_e))
            mind = length(tmp_e);
        end

        minds(ii) = mind;
        e(ii)     = tmp_e(mind);
        s(ii)     = tmp_s(mind);
    end
    
    % Active line segment as function of time

    if (should_plot_active_line_segment)

        figure();
        plot(tim, minds);
        xlabel('Time [s]');
        ylabel('Active line segment');
        title('Active line segment');
    end

    % Errors
    
    if (should_plot_along_track_errors)
        
        figure();
        plot(tim, s);
        xlabel('Time [s]');
        ylabel('Distance [m]');
        title('Along-track error');
    end
    
    if (should_plot_cross_track_errors)

        figure();
        plot(tim, e);
        xlabel('Time [s]');
        ylabel('Distance [m]');
        title('Cross-track error');
    end
end