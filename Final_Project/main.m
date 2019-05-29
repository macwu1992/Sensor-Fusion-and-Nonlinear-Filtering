clear all; clc;

showIP

run('startup.m')
%%

[xhat, meas] = myfilter()



%%


figure;
subplot(3,1,1)
plot(meas.gyr')
ylabel 'Gyro'

subplot(3,1,2)
plot(meas.mag')
ylabel 'Mag'

subplot(3,1,3)
plot(meas.acc')
ylabel 'Acc'

%%
valid_idx = ~isnan(meas.gyr(1,:));
gyro_data = meas.gyr(:,valid_idx)';

mean(gyro_data)
cov(gyro_data)

figure;
subplot(3,1,1)
hist(gyro_data(:,1), 50)
subplot(3,1,2)
hist(gyro_data(:,2), 50)
subplot(3,1,3)
hist(gyro_data(:,3), 50)
sgtitle('gyro')


valid_idx = ~isnan(meas.mag(1,:));
mag_data = meas.mag(:,valid_idx)';

mean(mag_data)
cov(mag_data)

figure;
subplot(3,1,1)
hist(mag_data(:,1), 50)
subplot(3,1,2)
hist(mag_data(:,2), 50)
subplot(3,1,3)
hist(mag_data(:,3), 50)
sgtitle('mag')


valid_idx = ~isnan(meas.acc(1,:));
acc_data = meas.acc(:,valid_idx)';

mean(acc_data)
cov(acc_data)

figure;
subplot(3,1,1)
hist(acc_data(:,1), 50)
subplot(3,1,2)
hist(acc_data(:,2), 50)
subplot(3,1,3)
hist(acc_data(:,3), 50)
sgtitle('acc')

%% 

q   = xhat.x(1:4,:);
b_w = xhat.x(5:7,:);
b_a = xhat.x(8:10,:);

plot(xhat.x())




