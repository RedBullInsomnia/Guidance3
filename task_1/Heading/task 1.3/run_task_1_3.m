%% Values for model
tstart=0;      %Sim start time
tstop=3000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[6.63 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

%% Relationship between rudder angle and yaw rate

% Define what rudder angles (d_c) to simulate for

d_c_limit        = 25 * (pi/180);
d_c_limit_detail = 1 * (pi/180);

number_of_tests = 20;
d_c_tests =            linspace(-d_c_limit,         -d_c_limit_detail,  number_of_tests);
d_c_tests = [d_c_tests linspace(-d_c_limit_detail,  d_c_limit_detail,   number_of_tests)];
d_c_tests = [d_c_tests linspace(d_c_limit_detail,   d_c_limit,          number_of_tests)];

r_tests = nan(size(d_c_tests));

% Simulate for the rudder inputs and collect steady state value for r

for i = 1:length(d_c_tests)
    d_c = d_c_tests(i);
    sim('MSFartoystyring');
    r_tests(i) = r(end);
end

%% Curve fit steady state response
p = @(c, x_data) (c(4) * x_data .^ 3) + (c(3) * x_data .^ 2) + (c(2) * x_data) + c(1);
c_0 = [-1 -1 -1 0]; % Guess at coefficients
c_ss = lsqcurvefit(p, c_0, r_tests, d_c_tests);

%% Fit a first order Nomoto
tstop = 2000;

d_c = 8 * (pi/180);
sim('MSFartoystyring');

f = @(c_f, x_data) c_f(1) * (1 - exp(-c_f(2) * x_data));
c_f_0 = [-0.005 0.005];

c_f = lsqcurvefit(f, c_f_0, t, r);
T = 1 / c_f(2);
K = c_f(1) / d_c;

%% Determing the values for the PID Heading controller 
% Model parameters
T = 80;
K = -0.041;
f = tf(K, [T 1 0]);

% figure
% margin (f)

% Controller, after book, page ...
% xsi = 1; % critical damping
% omega_b = 0.02; %bandwith of the system
% omega_n = omega_b /(sqrt(1 - 2*xsi^2 + sqrt(4*xsi^4 - 4*xsi^2 + 2)));
% 
% K_p = (T*omega_n^2)/K
% K_d = (2*xsi*omega_n*T - 1)/K
% K_i = K_p*omega_n/10
% 

psi_d = deg2rad(18);

tstop = 4000;
sim heading_model_controlled;

figure
plot(t, rad2deg(psi))
hold on
grid on
plot(get(gca,'xlim'), [rad2deg(psi_d) rad2deg(psi_d)], 'r--');
xlabel 'time (s)'
ylabel('Heading $\psi$ (rad)', 'Interpreter', 'latex')
