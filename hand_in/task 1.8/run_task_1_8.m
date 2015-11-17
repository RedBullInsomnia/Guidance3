clear all
close all
clc

tstart=0;      %Sim start time
tstop=2000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[4 0]';     %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=1;           %Current on (1)/off (0)

% Heading Controller gains 
Ki = 0.6;
Kp = 2000;
Kd = 100;
lf4 = -1.8257e+06;
lf1 = -500;
lf2 = 20;

% Surge controller gains
Kp_surge = -300;
Ki_surge = -0.01;

sim MSFartoystyring_1_8

% Figure 1 u and u_d
figure
plot(t, v(:,1), t, u_desired);
grid
xlabel 'time (s)'
ylabel 'velocity (m/s)'
legend('Surge speed', 'Desired surge speed');

% Figure 2 error
error = u_desired - v(:,1);
figure
plot(t, error)
grid
xlabel 'time (s)'
ylabel 'error (m/s)'