tstart=0;      %Sim start time
tstop=8000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[4 0]';     %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=1;           %Current on (1)/off (0)

sim MSFartoystyring_1_8

figure
plot(t, v(:,1), t, u_desired);
grid
xlabel 'time (s)'
ylabel 'velocity (m/s)'
legend('Surge speed', 'desired surge speed');

error = u_desired - v(:,1);
figure
plot(t, error)
grid
xlabel 'time (s)'
ylabel 'error (m/s)'