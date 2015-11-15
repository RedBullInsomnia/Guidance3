tstart=0;      %Sim start time
tstop=3000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[6.63 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=1;           %Current on (1)/off (0)


psi_d = -0.3*sin(0.008*t);
nc = 7.3;

sim MSFartoystyring_heading

figure(1)
plot(t, psi, t, psi_d, 'r')

xlabel('time(s)');
ylabel('Heading(rad)');
legend('\psi', '\psi_{desired}');