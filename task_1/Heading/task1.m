N = 20000;     % number of samples
h = 0.1;       % sample time
xout = zeros(N, 2);
x = zeros(6, 1);
x(1) = 8.23;
u = zeros(1, 2);
u(1) = -25*(pi/180);         % rudder angle step input
u(2) = 80*2*pi/60;           % 80 rpm

for i=1:N
    xout(i,:) = [(i-1)*h ,x(3)];
    xdot = msfartoystyring(x, u, 0);     % nonlinear Mariner model
    x = euler2(xdot, x, h);              % Euler integration
end

% time-series
tdata = xout(:,1);
rdata = xout(:,2);

%nonlinear least-squares parametrization: x(1)=1/T and x(2)=K
x0 = [50 1];
bech = @(x) 1872000*x.^3 -27.31*x;
F = @(x, tdata) x(2)*bech(-u(1))*(1 - exp(-tdata/x(1))); 
x = lsqcurvefit(F, x0, tdata, rdata);

% estimated parameters
T = x(1)
K = x(2)

figure
plot(tdata,rdata,'g',tdata, F([T K],tdata),'r'),grid
title('Nonlinear least-squares fit of MS Farstoystyring model for \delta = 5 (deg)'),xlabel('time (s)')
legend('Nonlinear model','Estimated 1st-order nonlinear Nomoto model')

%% Bech reverse spiral
tstart=0;      %Sim start time
tstop=1000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[6 0]';     %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

nc = 7.3;
for i=1:500
    dc = -20*(pi/180) + i*(4/50)*(pi/180);
    dc_stock(i) = dc;
    sim MSFartoystyring
    r_stock(i) = r(length(r));
end

%dc_stock = - dc_stock;
figure
plot(r_stock(1:500),dc_stock(1:500))

p1 = 1.872e+06;
p4 = -27.31; % course unstable ship

%%
tstart=0;      %Sim start time
tstop=1000;    %Sim stop time
tsamp=1;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[0.001 0]'; %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

nc = 7.3;
dc = 5*pi/180;

sim MSFartoystyring
rdata = r;
tdata = t;

x0 = [40 -200 -0.03]';
% Good values :p : [40 - 200 -0.02] or [39 -177 -0.02] 
f = @(x)odefun(x,rdata,tdata,dc);
%x = lsqnonlin(f, x0);

figure
plot(tdata, rdata*180/pi, tdata, model_heading(save, rdata, tdata, dc)*180/pi, 'r');
grid
legend('Nonlinear model','Estimated 2nd-order nonlinear Nomoto model')

%% Heading controller : 

% Model parameters :
K = -0.02;
T = 40 - 177;
f = tf([K], [T 1 0]);

figure
margin (f)

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