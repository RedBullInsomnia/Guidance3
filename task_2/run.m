
clc;
clear all;

tstart = 0;         %Sim start time
tstop  = 5000;      %Sim stop time
tsamp  = 10;        %Sampling time (NOT ODE solver time step)
track  = 0;
dec    = 10;

p0   = [0; 0];%zeros(2,1); %Initial position (NED)
v0   = [6.63 0]';   %Initial velocity (body)
psi0 = 0;           %Inital yaw angle
r0   = 0;           %Inital yaw rate
c    = 0;           %Current on (1)/off (0)

not_use_beta_transform  = 1;
not_use_speed_transform = 1;

sim MSFartoystyring

waypoints = waypoints_collection;

pathplotter(p(:, 1), p(:, 2),  psi, tsamp, dec, tstart, tstop, track, flipud(waypoints.waypoints));
return;

%%

figure(1);
clf();
hold on;

plot(p(:, 2), p(:, 1));
waypoints.plot_markers('k');
waypoints.plot_piecewise_continuous('k');
title('Path');
legend('MSFartøystyring', 'Path');
axis equal;

%

figure(2);
clf();
hold on;

plot(t, radtodeg(psi));
plot(t, radtodeg(psi_d(1, :)));
plot(t, radtodeg(chi_d(1, :)));
plot(t, radtodeg(beta));
legend('\psi', '\psi_d', '\chi_d', '\beta');

figure(5)
hold on;
plot(t, v);
plot(t, radtodeg(beta));

%

figure(3);
clf();

plot(t, waypoint_index(1, :));
title('Waypoint index');

%

figure(4);
clf();

plot(t, e(1, :));
title('Cross-track error');
