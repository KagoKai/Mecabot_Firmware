clc, clearvars, close all;
data = readtable("Motor_Vel_Log_50Hz.csv");
t = data.time;
amp = data.rpm_data;

t = t(1:3:end);
amp = amp(1:3:end);

order = 1;
fc = 20;
dt = max(diff(t));
fs = 50; % Sampling Frequency

[b, a] = butter(order,fc/(fs/2),"low");
freqz(b,a,1024,fs);

filtered_amp = zeros(length(amp),1);

for n=3:length(amp)
    filtered_amp(n) = -a(2) * filtered_amp(n-1) + b(1) * amp(n) + b(2) * amp(n-1);
    %filtered_amp(n) = -(a(2) * filtered_amp(n-1) + a(3) * filtered_amp(n-2)) + b(1) * amp(n) + b(2) * amp(n-1) + b(3) * amp(n-2);
end

plot(t, amp);
hold
plot(t, filtered_amp);