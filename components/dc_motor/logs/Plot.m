clc, clearvars, close all, format compact

% Load in data
data = readtable("./data/Motor_Vel_Log_10Hz.csv");

time = data.time;
rpm = data.rpm_data;

plot(time, rpm)
