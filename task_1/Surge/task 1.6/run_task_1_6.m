%% Determine the parameters for the model of surge

tstart=0;      %Sim start time
tstop=10000;   %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[0.001 0]'; %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

%% Fitting a first order transfer function
% sim MSFartoystyring_1_6
% rdata = v(:,1);
% 
% F = @(x, t) x(2)*nc*(1 - exp(-t/x(1))); 
% x0 = [400 1];
% x = lsqcurvefit(F, x0, t, rdata);
% 
% T = x(1)
% K = x(2)
% 
% nc = 80*pi/180;

%% Fitting the nonlinear model from book, page 141
nc = 80*2*pi/60;
psi_d = 0;

sim MSFartoystyring_1_6
rdata = v(:,1);
tdata = t;
sim surge_model

% 
figure
plot(t, rdata, 'g', t, u, 'r')
grid
xlabel 'time (s)'
ylabel 'speed (m/s)'
legend('MSFartoystyring','Estimated 1st-order linear model')

x0 = [3000 -1 -1];
f = @(x) odefun(x,rdata,tdata,nc);
x = lsqnonlin(f, x0);
x

% For nc =  1.39 rad/s, T=3531, xu = -0.3691, xu|u| = -0.4737
% For nc = 0.17 rad/s, T= 28141, xu = -0.37, xu|u| = -3.8087


%% Root locus for the gains, to be moved to task 1.7, if still used 
% T = 3000;
% K = 1.05;
% hw = tf(K, [T 1]);
% fb = tf([20 1], [1 0]);
% 
% cl = hw*fb;
% figure
% rlocus(cl, 0:0.0001:0.05);
% title ''
