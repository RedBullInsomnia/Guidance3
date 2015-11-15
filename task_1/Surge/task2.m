% N = 20000;     % number of samples
% h = 0.1;      % sample time
% xout = zeros(N, 2);
% x = zeros(6, 1);
% x(1) = 8.23;
% u = zeros(1, 2);
% u(1) = 0;                    % rudder angle
% u(2) = 80*2*pi/60;           % 80 rpm
% 
% for i=1:N
%     xout(i,:) = [(i-1)*h , x(4)];
%     xdot = msfartoystyring(x, u, 0);     % nonlinear Mariner model
%     x = euler2(xdot, x, h);              % Euler integration
% end
% 
% % time-series
% tdata = xout(:,1);
% rdata = xout(:,2);
% 
% % nonlinear least-squares parametrization: x(1)=1/T and x(2)=K
% x0 = [20 1]';
% F = @(x, tdata) -u(2)/x(2)*(exp(-x(2)/x(1)*tdata)) + u(2)/x(2)*(1 - exp(-x(2)/x(1)*tdata)); 
% x = lsqcurvefit(F, x0, tdata, rdata);
% 
% % estimated parameters
% A = x(1)
% B = x(2)
% 
% figure
% plot(tdata,rdata,'g',tdata, F(x, tdata),'r')
% grid
% title('Nonlinear least-squares fit of MS Farstoystyring model for \delta = 5 (deg)'),xlabel('time (s)')
% legend('Nonlinear model','Estimated 1st-order nonlinear Nomoto model')
% 
% 

%%% Testing the input
tstart=0;      %Sim start time
tstop=8000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[0.01 0]';     %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

nc = 80*pi/180;


sim MSFartoystyring_1_6

rdata = v(:,1);
tdata = t;
F = @(x, t) x(2)*nc*(1 - exp(-t/x(1))); 
x0 = [400 1];
x = lsqcurvefit(F, x0, tdata, rdata);
T = x(1)
K = x(2)

figure
plot(tdata,rdata,'g',tdata, F(x, tdata),'r')
grid
title('Nonlinear least-squares fit of MS Farstoystyring model for \delta = 8 (deg)'),xlabel('time (s)')
legend('MSFartoystyring','Estimated 1st-order linear Nomoto model')

%%
T = 500;
K = 1.05;
hw = tf(K, [T 1]);
fb = tf([20 1], [1 0]);

cl = hw*fb;
figure
rlocus(cl, 0:0.0001:0.05);
title ''
