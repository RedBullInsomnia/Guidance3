%% Determine the parameters for the model of surge

tstart=0;      %Sim start time
tstop=10000;   %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[0.001 0]'; %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

%% Plotting relationship between nc and steady state surge speed
% i = 1;
% for nc=0:1:8.7
%     sim MSFartoystyring_1_6;
%     surge_speed(i,1) = v(length(v),1);
%     surge_speed(i,2) = nc;
%     i = i + 1;
%     disp(i);
% end
% 
% figure
% plot(surge_speed(:,1), surge_speed(:,2))
% grid on
% xlabel('nc (rad/s)');
% ylabel('u (m/s)');


%% Fitting a first order transfer function
nc = 80*2*pi/60;
sim MSFartoystyring_1_6
rdata = v(:,1);

F = @(x, t) x(2)*nc*(1 - exp(-t/x(1))); 
x0 = [400 1];
x = lsqcurvefit(F, x0, t, rdata);

T = x(1)
K = x(2)

tf_sys = tf(K, [T 1]);

%margin(tf_sys)

%% Trying to fit through recursion
% 
% x0 = [3000 -1 -1];
% f = @(x) odefun(x,rdata,tdata,nc);
% x = lsqnonlin(f, x0);
% x
% 
% % For nc =  1.39 rad/s, T=3531, xu = -0.3691, xu|u| = -0.4737
% % For nc = 0.17 rad/s, T= 28141, xu = -0.37, xu|u| = -3.8087
% % For nc = 80rpm = 8.37 rad/s, T = 589, xu = -0.3847, xu|u| = -0.0771
% 
% % Surge model is derived from this fitting.

%% Showing of results for the linear model
nc = 20*2*pi/60;
psi_d = deg2rad(0);

sim MSFartoystyring_1_6
sim surge_model_linear

% 
figure
plot(t, v(:,1), 'g', t, u, 'r')
grid
xlabel 'time (s)'
ylabel 'speed (m/s)'
legend('MSFartoystyring','Estimated 1st-order nonlinear model')

figure
plot(t, psi)
grid
xlabel 'time (s)'
ylabel 'heading (rad)'

