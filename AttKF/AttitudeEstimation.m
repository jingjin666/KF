clc;
clear;
close all;

rad2deg = 180/pi;
deg2rad = pi/180;
g = 9.80665;

% file_nkf = 'nkf1.csv';
% data_list_nkf = importdata(file_nkf);
% data_nkf = data_list_nkf.data;
% 
% data_num_nkf=round(size(data_nkf,1));
% j=0;
% timestamp_nkf = zeros(data_num_nkf,1);
% NKF_Roll = zeros(data_num_nkf,1);
% NKF_Pitch = zeros(data_num_nkf,1);
% for i=1:data_num_nkf
%     j=j+1;
%     timestamp_nkf(j,1) = data_nkf(i,1);
%     NKF_Roll(j,1) = data_nkf(i,2);
%     NKF_Pitch(j,1) = data_nkf(i,3);
% end


file_imu = 'imu_static.csv';
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

%% 初始化状态
gyro_bias_x = 0.01;
gyro_bias_y = 0.01;
gyro_bias_z = 0.01;
X = [0;0;0;gyro_bias_x;gyro_bias_y;gyro_bias_z];

%协方差
P = eye(6);

%过程噪声
angle_process_noise = 1E-6 * ones(1, 3);
gyro_bias_process_noise = 1E-6 * ones(1, 3);
Q = diag([angle_process_noise,gyro_bias_process_noise]);

%测量噪声
acc_measure_noise = 0.2 * ones(1, 3);
acc_measure_noise = [0.0170,0.0466,0.0157];
R = diag(acc_measure_noise);

%% KF速度估计
for k=1:data_num_imu
    if(k>1)
        deltaT = timestamp_imu(k,1)-timestamp_imu(k-1,1);
    else
        deltaT = 0;
    end
    deltaT = deltaT*1E-6;
    %disp(deltaT);
    
    gyroMeas_X = GYRO(k,1);
    gyroMeas_Y = GYRO(k,2);
    gyroMeas_Z = GYRO(k,3);
    
    accMeas_X = ACC(k,1);
    accMeas_Y = ACC(k,2);
    accMeas_Z = ACC(k,3);
    accData = [accMeas_X,accMeas_Y,accMeas_Z];
    
    [roll, pitch, yaw] = acc2euler(accData);
    AccAngle(k, :) = [roll,pitch,yaw];
    
    %% 预测
    %angle_k = angle_k-1 + (gyro - gyro_bias_k-1)*deltaT
    %gyro_bias_k = gyro_bias_k-1
    f1 = -deltaT * eye(3);
    F = [eye(3),f1;
         zeros(3),eye(3)];
    Bu = [deltaT*gyroMeas_X;
          deltaT*gyroMeas_Y;
          deltaT*gyroMeas_Z;
          0;
          0;
          0];
    %状态预测
    X_next = F*X + Bu;
    %disp(X_next');
    %协方差预测
    P_next = F*P*F' + Q;
    
    %% 更新
   
    %计算残差
    H = [1,0,0,0,0,0;
         0,1,0,0,0,0;
         0,0,1,0,0,0];
    Z = AccAngle(k, :)';
    Hx = H*X_next;
    S = Z - Hx;
    %disp(S);
   
    %计算卡尔曼增益
    SP = H*P_next*H' + R;
    K = P_next*H'*(SP^-1);
    
    %状态更新
    X = X_next + K*S;
    %协方差更新
    P = (eye(6) - K*H)*P_next;
    
    %disp(X');
    KF(k,1) = X(1)*rad2deg;
    KF(k,2) = X(2)*rad2deg;
    
    Acc(k,1) = AccAngle(k,1)*rad2deg;
    Acc(k,2) = AccAngle(k,2)*rad2deg;
end

figure;
%plot(timestamp_imu,Acc(:,2),timestamp_imu,KF(:,2));
%plot(timestamp_imu,Acc(:,2),timestamp_imu,ATT(:,2));
%plot(timestamp_imu,KF(:,2),timestamp_imu,ATT(:,2));
plot(timestamp_imu,Acc(:,2),timestamp_imu,KF(:,2),timestamp_imu,ATT(:,2));
legend('ACC','KF','NKF','FontSize',10);
xlabel('Time(us)','FontSize',20);
ylabel('Angle(deg)','FontSize',20);
title('Attitude Pitch','FontSize',20);

figure;
plot(timestamp_imu,Acc(:,1),timestamp_imu,KF(:,1),timestamp_imu,ATT(:,1));
legend('ACC','KF','NKF','FontSize',10);
xlabel('Time(us)','FontSize',20);
ylabel('Angle(deg)','FontSize',20);
title('Attitude Roll','FontSize',20);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NED
function [roll, pitch, yaw] = acc2euler(acce_data)
    ax = acce_data(1)/norm(acce_data);
    ay = acce_data(2)/norm(acce_data);
    az = acce_data(3)/norm(acce_data);
    roll = atan2(-ay, -az);
    pitch = atan2(ax, sqrt(ay*ay + az*az));
    yaw = 0;
end