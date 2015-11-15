function r = first_order_nomoto_simulator(model_parameters, H_b_c, r_0, t, d_c)

%% Simulation of first order Nomoto model

T = model_parameters(1);
K = model_parameters(2);

u  = d_c;

% Steady state relationship

H_b = @(r) H_b_c(4) * r .^ 3 + H_b_c(3) * r .^ 2 + H_b_c(2) * r + H_b_c(1);

% T * r´ + K * Hb(r) = K * u

first_order_nomoto = @(t, r) (1 / T) * (K * u - K * H_b(r));

% Simulate model with fixed step solver and the interpolate data to match
% time series from simulink simulation

sim_timespan   = [min(t), max(t)];
[sim_t, sim_r] = ode45(first_order_nomoto, sim_timespan, r_0);

r = interp1(sim_t, sim_r, t);

end
