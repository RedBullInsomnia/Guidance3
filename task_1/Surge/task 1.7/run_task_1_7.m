%% Root locus, to gain some insight into the values that should be used
T = 500;
K = 1.05;
hw = tf(K, [T 1]);
fb = tf([20 1], [1 0]);

cl = hw*fb;
figure
rlocus(cl, 0:0.0001:0.05);
title ''
