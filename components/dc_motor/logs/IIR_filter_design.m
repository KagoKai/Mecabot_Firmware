clc, clearvars, close all;
data = readtable("./data/Motor_Vel_Log_10Hz.csv");
t = data.time;
amp = data.rpm_data;

%t = t(1:3:end);
%amp = amp(1:3:end);

order = 1;
fc = 100;
dt = max(diff(t));
fs = 1000; % Sampling Frequency

[b1, a1] = butter(order,fc/(fs/2),"low");
[b2, a2] = butter(2, fc/(fs/2), "low");
freqz(b1,a1,1024,fs);

filtered_amp = zeros(length(amp),1);

for n=2:length(amp)
    filtered_amp(n) = -a1(2) * filtered_amp(n-1) + b1(1) * amp(n) + b1(2) * amp(n-1);
    %filtered_amp(n) = -(a(2) * filtered_amp(n-1) + a(3) * filtered_amp(n-2)) + b(1) * amp(n) + b(2) * amp(n-1) + b(3) * amp(n-2);
end

plot(t, amp);
hold;
plot(t, filtered_amp);