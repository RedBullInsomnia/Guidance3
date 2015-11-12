clear all
close all
clc

T = 117.52; % Time constant
K = -3.54;  % Gain

f = tf(K, [T 1])*tf(1, [1 0]) % Transfer function

