clc;
clear;
close all;

rad2deg = 180/pi;
deg2rad = pi/180;
g = 9.80665;

file_imu = 'imu_R.csv';
data_list_imu = importdata(file_imu);
imu_time_list = data_list_imu.textdata;
data_imu = data_list_imu.data;

data_num_imu=round(size(data_imu,1));
j=0;
timestamp_imu = zeros(data_num_imu,1);
GYRO = zeros(data_num_imu,3);
ACC = zeros(data_num_imu,3);
ATT = zeros(data_num_imu,2);
for i=1:data_num_imu
    j=j+1;
    timestamp_imu(j,1) = data_imu(i,1);
    GYRO(j,1) = data_imu(i,2);
    GYRO(j,2) = data_imu(i,3);
    GYRO(j,3) = data_imu(i,4);
    ACC(j,1) = data_imu(i,5);
    ACC(j,2) = data_imu(i,6);
    ACC(j,3) = data_imu(i,7);
    ATT(j,1) = data_imu(i,8);
    ATT(j,2) = data_imu(i,9);
end

std_variance_x = std(ACC(:,1),1); 
disp((3*std_variance_x)^2);
std_variance_y = std(ACC(:,2),1); 
disp((3*std_variance_y)^2);
std_variance_z = std(ACC(:,3),1); 
disp((3*std_variance_z)^2);

figure;
plot(timestamp_imu,ACC(:,1),timestamp_imu,ACC(:,2),timestamp_imu,ACC(:,3));
legend('ACC-X','ACC-Y','ACC-Z','FontSize',10);
xlabel('Time(us)','FontSize',20);
ylabel('G','FontSize',20);
title('º”ÀŸ∂»º∆‘Î…˘','FontSize',20);

figure;
plot(timestamp_imu,GYRO(:,1),timestamp_imu,GYRO(:,2),timestamp_imu,GYRO(:,3));
legend('GYRO-X','GYRO-Y','GYRO-Z','FontSize',10);
xlabel('Time(us)','FontSize',20);
ylabel('R','FontSize',20);
title('Õ”¬›“«‘Î…˘','FontSize',20);