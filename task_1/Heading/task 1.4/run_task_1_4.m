tstart=0;      %Sim start time
tstop=3000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[6.63 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=1;           %Current on (1)/off (0)

t = 0:10:3000;
psi_d(:,2) = -0.3*sin(0.008*t);
psi_d(:,1) = t;
nc = 7.3;

% K_i was -0.00001 at first (and on the saved graphs)

sim MSFartoystyring_1_4

% Figure 1 psi~
err = psi_d(:,2) - psi;
figure
plot(t,err)
xlabel 'time(s)'
ylabel ('$\tilde{\psi}$(rad)','Interpreter', 'latex')

% Figure 2 psi and psi_d
figure
plot(t, psi, t, psi_d(:,2), 'r')

xlabel 'time(s)'
ylabel 'Heading(rad)'
legend('\psi', '\psi_{desired}');

% Figure 3 r~
r_d = -0.008*0.3*cos(0.008*t);
err_r = r_d - r;
figure
plot(t, err_r)
xlabel 'time(s)'
ylabel ('$\tilde{r}$(rad/s)','Interpreter', 'latex')

% Figure 4 r and r_d
figure
plot(t, r, t, r_d)
xlabel 'time(s)'
ylabel 'yaw rate(rad/s)'
legend('r', 'r_{desired}')