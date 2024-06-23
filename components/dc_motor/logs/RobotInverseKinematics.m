clc, clearvars, close all;

WHEEL_SEPARATION_X = 0.115;
WHEEL_SEPARATION_Y = 0.138;
R = 0.03;

vx = 0.5;
vy = 0;
wz = 0;

C = (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) / 2;

w_FL = (1/R) * (vx - vy - C * wz);
w_FR = (1/R) * (vx + vy + C * wz);
w_BL = (1/R) * (vx + vy - C * wz);
w_BR = (1/R) * (vx - vy + C * wz);