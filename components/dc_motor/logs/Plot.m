clc, clearvars, close all, format compact

% Load in data
data = readtable("./data/IMU_Accel_Log_20Hz.csv");

time = data.time;
ax = data.ax;

plot(time, ax);
