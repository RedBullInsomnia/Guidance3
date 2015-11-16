%% Root locus, to gain some insight into the values that should be used
T = 500;
K = 0.98;
hw = tf(K, [T 1]);
fb = tf([200 1], [1 0]);

cl = hw*fb;
figure
rlocus(cl, 0:0.0001:0.05);
title ''

%% Testing the controller
u_d = 4;
sim surge_model_controlled;

figure
plot(t, u)
hold on;
plot(get(gca,'xlim'), [u_d u_d], 'r--');