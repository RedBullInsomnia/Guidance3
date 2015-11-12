clear all;

tstart=0;      %Sim start time
tstop=2000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[1 0]';   %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=1;           %Current on (1)/off (0)

%% Heading controller : 

% Model parameters :
K = 3;
T = 117;
f = tf([K 0], [T 1 0]);

% figure
% margin (f)

% Controller :
xsi = 0.707;
omega_b = 0.32;
T = 117.52;
K = 3.54;
omega_n = omega_b /(sqrt(1 - 2*xsi^2 + sqrt(4*xsi^4 - 4*xsi^2 + 2)));

K_p = (T*omega_n^2)/K;
K_d = (2*xsi*omega_n*T - 1)/K;
%K_i = K_p*omega_n/10;
K_i = 0;

%% Speed controller :

% Model parameters : 
T2 = 351.28/0.96;
K2 = 1/0.96;

f2 = tf(K2, [T2 1]);

%figure(2)
%bode(f2)

% Controller :
K_p2 = 20;
K_i2 = 0.005;

%% Simulation

% Controls :
u_desired = 7.3;
psi_desired = deg2rad(0);

sim MSFartoystyring

%% Plots

% f(t) = psi :
figure
plot(t, rad2deg(psi))
xlabel('time(s)');
ylabel('psi(deg)');
title('\psi function of the time');

% f(t) = u :
% figure
% plot(t, v(:,1));
% xlabel('time(s)');
% ylabel('u(m/s)');
% title('u function of the time');

% % f(t) = v :
% figure(5)
% plot(t, v(:,2));
% xlabel('x(m)');
% ylabel('y(m)');

% f(x) = y :
figure
plot(x, y);
xlabel('x(m)');
ylabel('y(m)');
title('Position of the ship');
axis equal

%% Code to find the surge model :
% u_stock=zeros(1,200);
% for i=1:1:84
%     nc_stock(i) = i/10;
%     nc = i/10;
%     sim MSFartoystyring3;
%     u_stock(i)= v(length(v),1);
% end
% plot(u_stock);
% %Curve fit