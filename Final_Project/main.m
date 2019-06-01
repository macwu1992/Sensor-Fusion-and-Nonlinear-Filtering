clear all; clc;

run('startup.m')
showIP


%%

[xhat, meas] = myfilter()



%% Task 2-a

load(fullfile(pwd,'/logdata/Calibration.mat'))


close all;


figure('Color','white','Position',[369  172  989  669]);

subplot(3,1,1)
hold on; grid on; 
plot(meas.t', meas.gyr', 'LineWidth', 1)
xlim([0, max(meas.t)])
title 'Gyroscope', ylabel 'angular velocity [rad/s]', xlabel 'time [s]'
legend({'x','y','z'})

subplot(3,1,2)
hold on; grid on;
plot(meas.t', meas.mag', 'LineWidth', 2)
xlim([0, max(meas.t)])
% ylim([-1 11]);
title 'Magnetometer', ylabel 'magnetic field [uT]', xlabel 'time [s]'
legend({'x','y','z'})

subplot(3,1,3)
hold on; grid on;
plot(meas.t', meas.acc', 'LineWidth', 2)
xlim([0, max(meas.t)])
ylim([-1 11]);
title 'Accelerometer', ylabel 'acceleration [m/s^2]', xlabel 'time [s]'
legend({'x','y','z'})

% fp.savefig('Task2-plots')



%% Task 2-b

load(fullfile(pwd,'/logdata/Calibration.mat'))


close all;
axname = {'x','y','z'};

valid_idx = ~isnan(meas.gyr(1,:));
gyro_data = meas.gyr(:,valid_idx)';

mu   = mean(gyro_data)
Rw = cov(gyro_data)

figure('Color','white');
for i=1:3
    subplot(3,1,i); hold on; grid on;
    histogram(gyro_data(:,i), 'Binwidth', 1.01e-3, 'Normalization','pdf')
    xlabel([axname{i},' [rad/s]']);
    
    [x,y] = normpdf2(mu(i), Rw(i,i), 3, 100);
    plot(x,y, 'LineWidth',2, 'DisplayName', sprintf('gaussian N(x; 0, $P_{N|N})$') );
            
end
sgtitle('Histogram gyroscope - angular velocity')
fp.savefig('Task2-hist-gyro')




valid_idx = ~isnan(meas.mag(1,:));
mag_data = meas.mag(:,valid_idx)';

mu   = mean(mag_data)
Rm = cov(mag_data)

figure('Color','white');
for i=1:3
    subplot(3,1,i); hold on; grid on;
    histogram(mag_data(:,i), 'Binwidth', 1.01e-1, 'Normalization','pdf')
    xlabel([axname{i},' [uT]']);
    
    [x,y] = normpdf2(mu(i), Rm(i,i), 3, 100);
    plot(x,y, 'LineWidth',2, 'DisplayName', sprintf('gaussian N(x; 0, $P_{N|N})$') );
            
end
sgtitle('Histogram magnetometer - magnetic field')
fp.savefig('Task2-hist-mag')



valid_idx = ~isnan(meas.acc(1,:));
acc_data = meas.acc(:,valid_idx)';

mu   = mean(acc_data)
Ra = cov(acc_data)

figure('Color','white');
for i=1:3
    subplot(3,1,i); hold on; grid on;
    histogram(acc_data(:,i), 50, 'Normalization','pdf')
    xlabel([axname{i},' [m/s^2]']);
    
    [x,y] = normpdf2(mu(i), Ra(i,i), 3, 100);
    plot(x,y, 'LineWidth',2, 'DisplayName', sprintf('gaussian N(x; 0, $P_{N|N})$') );
            
end
sgtitle('Histogram accelerometer - acceleration')
fp.savefig('Task2-hist-acc')





%% Question 12


close all;


name = 'allsensors_01';
% load(fullfile(pwd,'/logdata/allsensors_01.mat'))
tmin = 20;
tmax = 70;

name = 'nogyro';
tmin=0;
tmax=50;

name = 'noacc';
tmin=0;
tmax=42;

name = 'nomag';
tmin=0;
tmax=42;

name = 'onlygyro';
tmin=0;
tmax=30;


load(fullfile(pwd, ['/logdata/',name,'.mat']))
idx = xhat.t>tmin & xhat.t < tmax;

figure('Color','white','Position',[364   363  1258   473]);
sgtitle('Orientation estimation - comparison Google vs Own')

subplot(3,1,1)
hold on; grid on; 
plot(xhat.t(idx)', 180/pi*([1 0 0]*q2euler(xhat.x(:,idx)))',      'LineWidth', 2)
plot(xhat.t(idx)', 180/pi*([1 0 0]*q2euler(meas.orient(:,idx)))', 'LineWidth', 2)
xlim([tmin, tmax])
ylabel 'phi [deg]', xlabel 'time [s]'
legend({'Own','Google'})

subplot(3,1,2)
hold on; grid on; 
plot(xhat.t(idx)', 180/pi*([0 1 0]*q2euler(xhat.x(:,idx)))',      'LineWidth', 2)
plot(xhat.t(idx)', 180/pi*([0 1 0]*q2euler(meas.orient(:,idx)))', 'LineWidth', 2)
xlim([tmin, tmax])
ylabel 'theta [deg]', xlabel 'time [s]'
legend({'Own','Google'})


subplot(3,1,3)
hold on; grid on; 
plot(xhat.t(idx)', 180/pi*([0 0 1]*q2euler(xhat.x(:,idx)))',      'LineWidth', 2)
plot(xhat.t(idx)', 180/pi*([0 0 1]*q2euler(meas.orient(:,idx)))', 'LineWidth', 2)
xlim([tmin, tmax])
ylabel 'psi [deg]', xlabel 'time [s]'
legend({'Own','Google'})

fp.savefig(['Task11_',name])


%% help functions


function [x,y] = normpdf2(mu, sigma2, level, N)
    x = linspace(mu-level*sqrt(sigma2), mu+level*sqrt(sigma2), N);
    y = normpdf(x, mu, sqrt(sigma2));
end


