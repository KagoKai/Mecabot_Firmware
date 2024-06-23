clc, clearvars, close all;

C = sym('C');

V = 3;
I = 0.07;
n = 110;

R = (V - n * C) / I;

% 3V speed
n_3v = (3 - I * R) / C;
% 5V speed
n_5v = (5 - I * R) / C;
% 10V speed
n_10v = (10 - I*R) / C;