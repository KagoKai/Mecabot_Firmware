clc, clearvars, close all;
data = readtable("./data/Motor_Vel_Log_10Hz.csv");
t = data.time;
amp = data.rpm_data;

% t = t(1:3:end);
% amp = amp(1:3:end);

dt = max(diff(t));
Fs = 1000 ; % Sampling Frequency

L = length(t);
Y = fft(amp);

%P2 = abs(Y/L);
%P1 = P2(1:L/2+1);
%P1(2:end-1) = 2*P1(2:end-1);

plot(Fs/L*(-L/2:L/2-1),abs(fftshift(Y/L)),"LineWidth",3)

%f = Fs*(0:(L/2))/L;
%plot(f,P1,"LineWidth",3) 
%title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")

xlim([0 10])
%ylim([0 50]) 