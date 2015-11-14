tstart=0;      %Sim start time
tstop=8000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[4 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=1;           %Current on (1)/off (0)

%psi_desired = -0.3*sin(0.008*t);
psi_desired = 0;

sim MSFartoystyring_1_8

plot(t, v(:,1));
xlabel('time(s)');
ylabel('u(m/s)');
title('u function of the time');