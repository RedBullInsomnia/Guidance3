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

%% Plot relationship between rudder angle and yaw rate

figure(1);
hold on;

plot(radtodeg(d_c_tests), r_tests, 'x-');

xlabel('\delta_c');
ylabel('r');
title('Steady state relationship between rudder angle and yaw rate');

%% Curve fit steady state response

p = @(c, x_data) (c(4) * x_data .^ 3) + (c(3) * x_data .^ 2) + (c(2) * x_data) + c(1);
c_0 = [-1 -1 -1 0]; % Guess at coefficients

c_ss = lsqcurvefit(p, c_0, r_tests, d_c_tests);

plot(radtodeg((c_ss(4) * r_tests .^ 3) + (c_ss(3) * r_tests .^ 2) + (c_ss(2) * r_tests) + c_ss(1)), r_tests);

%% Fit a first order Nomoto

tstop = 2000;

d_c = 8 * (pi/180);
sim('MSFartoystyring');

f = @(c_f, x_data) c_f(1) * (1 - exp(-c_f(2) * x_data));
c_f_0 = [-0.005 0.005];

c_f = lsqcurvefit(f, c_f_0, t, r);

figure(2);
clf();
hold on;

plot(t, r);
plot(t, c_f(1) * (1 - exp(-c_f(2) * t)));

T = 1 / c_f(2);
K = c_f(1) / d_c;

disp(['Nomoto model: ' num2str(T) 'r^ + r = ' num2str(K) '\delta']);

%% Testing of linear model

d_c = 8 * (pi/180);

sim('MSFartoystyring');
sim('nomoto_model');

figure(3);
hold on;

plot(t, r);
plot(nomoto_t, nomoto_r);

legend('MSFartoystyring', 'Nomoto');

%% Testing of nonlinear model

d_c = 1 * (pi/180);

sim('MSFartoystyring');
sim('nomoto_model');
sim('nomoto_model_nonlinear');

figure(4);
hold on;

plot(t, r);
plot(nomoto_t, nomoto_r);
plot(nonlinear_nomoto_t, nonlinear_nomoto_r);
legend('MSFartoystyring', 'Linear Nomoto', 'Nonlinear Nomoto');

