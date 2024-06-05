clc, clearvars, close all, format compact

% Load in data
data = readtable("Motor_Vel_Log_50Hz.csv");

time = data.time;
rpm = data.rpm_data;

plot(time, rpm)
