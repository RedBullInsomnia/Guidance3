clc;
clear all;

%% Basic simulation parameters

tstart=0;      %Sim start time
tstop=3000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[6.63 0]';  %Initial velocity (body)
psi0=0;          %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

%% Obtain relationship between rudder angle and yaw rate

% Define what rudder angles (d_c) to simulate for

d_c_limit        = 25 * (pi/180);
d_c_limit_detail = 8 * (pi/180);

number_of_tests = 20;
d_c_tests =            linspace(-d_c_limit,         -d_c_limit_detail,  number_of_tests);
d_c_tests = [d_c_tests linspace(-d_c_limit_detail,  d_c_limit_detail,   number_of_tests)];
d_c_tests = [d_c_tests linspace(d_c_limit_detail,   d_c_limit,          number_of_tests)];

r_tests = nan(size(d_c_tests));

% Simulate for the rudder inputs and collect steady state value for r

for i = 1:length(d_c_tests)
    d_c = d_c_tests(i);
    sim('MSFartoystyring_HeadingIdentification');
    r_tests(i) = r(end);
end

%% Plot relationship between rudder angle and yaw rate

figure();
hold on;

plot(radtodeg(d_c_tests), r_tests, 'x-');

xlabel('\delta_c');
ylabel('r');
title('Steady state relationship between rudder angle and yaw rate');

%% Curve fit steady state response

% Fit a third order polynomial

p = @(c, x_data) (c(4) * x_data .^ 3) + (c(3) * x_data .^ 2) + (c(2) * x_data) + c(1);
H_b_c_0 = [-1 -1 -1 0];

% Fit steady state coefficients

H_b_c = lsqcurvefit(p, H_b_c_0, r_tests, d_c_tests);

plot(radtodeg((H_b_c(4) * r_tests .^ 3) + (H_b_c(3) * r_tests .^ 2) + (H_b_c(2) * r_tests) + H_b_c(1)), r_tests);

%% Setup for simulations for fitting models

tstop = 2000;

d_c = degtorad(8);

sim('MSFartoystyring_HeadingIdentification');

%% First order linear Nomoto model

% Fitting

f = @(c_f, x_data) c_f(1) * (1 - exp(-c_f(2) * x_data));
c_f_0 = [-0.005 0.005];

c_f = lsqcurvefit(f, c_f_0, t, r);

% Define model

T = 1 / c_f(2);
K = c_f(1) / d_c;

disp(['Nomoto model: ' num2str(T) 'r^ + r = ' num2str(K) '\delta']);

% Simulation comparison

nomoto_linear_1_r = d_c * K * (1 - exp(-(1 / T) * t));

figure();
hold on;

plot(t, r);
plot(t, nomoto_linear_1_r);

legend('MSFartoystyring', 'First order linear Nomoto');

%% First order nonlinear Nomoto model

% Fitting

f = @(model_parameters, x_data) first_order_nomoto_simulator(model_parameters, H_b_c, 0, t, d_c);

c_f_0 = [1000 -0.5];
c_f = lsqcurvefit(f, c_f_0, t, r);

% Simulation comparison

nomoto_nonlinear_1_r = first_order_nomoto_simulator(c_f, H_b_c, zeros(2, 1), t, d_c);

figure();
hold on;

plot(t, r);
plot(t, nomoto_linear_1_r);
plot(t, nomoto_nonlinear_1_r);

legend('MSFartoystyring', 'First order linear Nomoto', 'First order nonlinear Nomoto');

%% Fitting second order nonlinear model

% !!!NOTE!!!
% This does not work properly. Coupled T1 and T2 and
% dominating damping terms are the reason for this is what I
% think at least. Look at the coefficient in H_b, r^3 is huge.

% Fitting

f = @(model_parameters, x_data) second_order_nomoto_simulator(model_parameters, H_b_c, zeros(2, 1), t, d_c);

c_f_0 = [1, 1, -0.0002];
c_f = lsqcurvefit(f, c_f_0, t, r);

% Simulation comparison

nomoto_nonlinear_2_r = second_order_nomoto_simulator(c_f, H_b_c, zeros(2, 1), t, d_c);

figure(1);
clf();
hold on;

plot(t, r);
plot(t, nomoto_linear_1_r);
plot(t, nomoto_nonlinear_1_r);
plot(t, nomoto_nonlinear_2_r);

legend('MSFartoystyring', 'First order linear Nomoto', 'First order nonlinear Nomoto', 'Second order nonlinear Nomoto');
