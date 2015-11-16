function r = second_order_nomoto_simulator(model_parameters, H_b_c, r_0, t, d_c)

%% Simulation of second order Nomoto model

T1 = model_parameters(1);
T2 = model_parameters(2);
K  = model_parameters(3);

u  = d_c;

% Steady state relationship

H_b = @(r) H_b_c(4) * r .^ 3 + H_b_c(3) * r .^ 2 + H_b_c(2) * r + H_b_c(1);

% (T1 * T2) r'' + (T1 + T2) r´ + K * Hb(r) = K  u + (K * T3) u'

% Neglect u', which gives simulation model

% x1 = r
% x2 = r'

% x1 = x2
% x2 = (1 / (T1 * T2)) * (K u - (T1 + T2) r´ + K H_b(r))

second_order_nomoto_x1 = @(t, x) x(2);
second_order_nomoto_x2 = @(t, x) (1 / (T1 * T2)) * (K * u - (T1 + T2) * x(2) - K * x(1) - K * H_b(x(1)));

second_order_nomoto    = @(t, x) [second_order_nomoto_x1(t, x), second_order_nomoto_x2(t, x)]';

% Simulate model with fixed step solver and the interpolate data to match
% time series from simulink simulation

sim_timespan   = [min(t), max(t)];
[sim_t, sim_x] = ode45(second_order_nomoto, sim_timespan, r_0);

sim_x          = sim_x';                            % Vectors vertically, for intuition
r              = interp1(sim_t, sim_x(1, :), t);

end
