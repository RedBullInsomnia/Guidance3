N = 20000;     % number of samples
h = 0.1;      % sample time
xout = zeros(N, 2);
x = zeros(6, 1);
x(1) = 8.23;
u = zeros(1, 2);
u(1) = -15*(pi/180);         % rudder angle step input
u(2) = 80*2*pi/60;           % 80 rpm

for i=1:N
    xout(i,:) = [(i-1)*h ,x(3)];
    xdot = msfartoystyring(x, u, 0);     % nonlinear Mariner model
    x = euler2(xdot, x, h);              % Euler integration
end

% time-series
tdata = xout(:,1);
rdata = xout(:,2)*180/pi;

%nonlinear least-squares parametrization: x(1)=1/T and x(2)=K
x0 = [0.1 1]';
F = @(x, tdata) exp(-tdata*x(1))*0 + (-x(2)*u(1))*(1 - exp(-tdata*x(1))); 
x = lsqcurvefit(F, x0, tdata, rdata);

% estimated parameters
T = 1/x(1)
K = x(2)

figure
plot(tdata,rdata,'g',tdata,exp(-tdata/T)*0 + (-K*u(1))*(1 - exp(-tdata/T)),'r'),grid
title('Nonlinear least-squares fit of MS Farstoystyring model for \delta = 5 (deg)'),xlabel('time (s)')
legend('Nonlinear model','Estimated 1st-order nonlinear Nomoto model')
