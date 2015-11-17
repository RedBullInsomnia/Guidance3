
clc;
clear all;
close all;

%% Simulation

tstart = 0;         %Sim start time
tstop  = 4500;      %Sim stop time
tsamp  = 10;        %Sampling time (NOT ODE solver time step)
dec    = 10;

p0   = zeros(2,1);  %Initial position (NED)
v0   = [6.63 0]';   %Initial velocity (body)
psi0 = 0;           %Inital yaw angle
r0   = 0;           %Inital yaw rate
c    = 1;           %Current on (1)/off (0)

track  = 1;

not_use_beta_transform  = 0;
not_use_speed_transform = 0;

%% Load heading, surge controller parameters

load('heading_controller');
load('surge_controller');

%% Simulate

sim('MSFartoystyring_task_2_7');

%% Results

waypoints = waypoints_collection;
pathplotter_original(p(:, 1), p(:, 2),  psi, tsamp, dec, tstart, tstop, track, waypoints.waypoints);
 