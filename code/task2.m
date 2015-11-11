N = 20000;     % number of samples
h = 0.1;      % sample time
xout = zeros(N, 2);
x = zeros(6, 1);
x(1) = 8.23;
u = zeros(1, 2);
u(1) = 0;                    % rudder angle
u(2) = 80*2*pi/60;           % 80 rpm

for i=1:N
    xout(i,:) = [(i-1)*h , x(1)];
    xdot = msfartoystyring(x, u, 0);     % nonlinear Mariner model
    x = euler2(xdot, x, h);              % Euler integration
end

% time-series
tdata = xout(:,1);
rdata = xout(:,2);

% nonlinear least-squares parametrization: x(1)=1/T and x(2)=K
x0 = [20 1]';
F = @(x, tdata) -u(2)/x(2)*(exp(-x(2)/x(1)*tdata)) + u(2)/x(2)*(1 - exp(-x(2)/x(1)*tdata)); 
x = lsqcurvefit(F, x0, tdata, rdata);

% estimated parameters
A = x(1)
B = x(2)

figure
plot(tdata,rdata,'g',tdata, F(x, tdata),'r')
grid
title('Nonlinear least-squares fit of MS Farstoystyring model for \delta = 5 (deg)'),xlabel('time (s)')
legend('Nonlinear model','Estimated 1st-order nonlinear Nomoto model')