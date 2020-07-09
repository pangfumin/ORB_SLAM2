% plot_gyro.m

clear all;
close all;
clc;

data = load('gyros.txt');

gyro0 = data(:,1:3);
gyro1 = data(:,4:6);

t = 1: size(gyro0,1);
figure(1)

subplot(3,1,1);
plot(t, gyro0(:,1),'r', t, gyro1(:,1),'g');

subplot(3,1,2);
plot(t, gyro0(:,2),'r', t, gyro1(:,2),'g');

subplot(3,1,3);
plot(t, gyro0(:,3),'r', t, gyro1(:,3),'g');

figure(2);
subplot(3,1,1)
[c,lags] = xcorr(gyro0(:,1),gyro1(:,1));
plot(lags, c);
hold on;
[m, idx] = max(c);
max_idx = lags(idx);
plot(max_idx, m, '*');
max_idx
text(max_idx,m,['(' num2str(max_idx) ',' num2str(m) ')'])
grid on;
xlabel('#imu frame')
title('Angular Velocity Correlation x');

subplot(3,1,2)
[c,lags] = xcorr(gyro0(:,2),gyro1(:,2));
plot(lags, c);
hold on;
[m, idx] = max(c);
max_idx = lags(idx);
plot(max_idx, m, '*');
max_idx
text(max_idx,m,['(' num2str(max_idx) ',' num2str(m) ')'])
grid on;
xlabel('#imu frame')
title('Angular Velocity Correlation y');


subplot(3,1,3)
[c,lags] = xcorr(gyro0(:,3),gyro1(:,3));
plot(lags, c);
hold on;
[m, idx] = max(c);
max_idx = lags(idx);
plot(max_idx, m, '*');
max_idx
text(max_idx,m,['(' num2str(max_idx) ',' num2str(m) ')'])
grid on;
xlabel('#imu frame')
title('Angular Velocity Correlation z');