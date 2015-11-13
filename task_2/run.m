
clc;
clear all;

%% Simulation

tstart = 0;         %Sim start time
tstop  = 5000;      %Sim stop time
tsamp  = 10;        %Sampling time (NOT ODE solver time step)
dec    = 10;

p0   = zeros(2,1);  %Initial position (NED)
v0   = [6.63 0]';   %Initial velocity (body)
psi0 = 0;           %Inital yaw angle
r0   = 0;           %Inital yaw rate
c    = 1;           %Current on (1)/off (0)

not_use_beta_transform  = 0;
not_use_speed_transform = 1;

track  = 1;

p0 = [0; 0];

sim('MSFartoystyring');

%% Results

u = v(:, 1);
v = v(:, 2);

if (true)
    
    waypoints = waypoints_collection;
    pathplotter(p(:, 1), p(:, 2),  psi, tsamp, dec, tstart, tstop, track, waypoints.waypoints, u, v);
end
    
figureIndex = 12;

% Speed

if (true)

	figure(figureIndex);
	clf();
	figureIndex = figureIndex + 1;
	hold on;

	plot(t, U_d(1, :));
	plot(t, sqrt(u .^ 2 + v .^ 2));
    plot(t, u);
    plot(t, u_d(1, :));
    plot(t, v);
	title('Speed');
	legend('U_d', 'U', 'u', 'u_d', 'v');
end

% Inputs

if (true)

	figure(figureIndex);
	clf();
	figureIndex = figureIndex + 1;
	hold on;

	plotyy(t, d_c, t, n_c(1, :));

	title('Inputs');
end

% Heading

if (true)

	figure(figureIndex);
	clf();
	figureIndex = figureIndex + 1;
	hold on;

	plot(t, radtodeg(psi));
	plot(t, radtodeg(psi_d(1, :)));

	legend('\psi', '\psi_d');
end

% Course

if (true)

	figure(figureIndex);
	clf();
	figureIndex = figureIndex + 1;
	hold on;

	plot(t, radtodeg(chi_d(1, :)));
	plot(t, radtodeg(psi) + radtodeg(beta));
	legend('\chi_d', '\chi');
end

% Crab angle

if (true)

	figure(figureIndex);
	figureIndex = figureIndex + 1;
	hold on;
    
	plot(t, v);
	plot(t, radtodeg(beta));
    
    title('Crab angle');
    legend('v', '\beta');
end

% Waypoint index

if (false)

	figure(figureIndex);
	clf();
	figureIndex = figureIndex + 1;

	plot(t, waypoint_index(1, :));
	title('Waypoint index');
end

% Cross track

if (false)

	figure(figureIndex);
	clf();
	figureIndex = figureIndex + 1;

	plot(t, e(1, :));
	title('Cross-track error');
end
