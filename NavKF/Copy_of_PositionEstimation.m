clc;
clear;
close all;

file_imu = 'imu_nav_sitl2.csv';
data_list_imu = importdata(file_imu);
imu_time_list = data_list_imu.textdata;
data_imu = data_list_imu.data;

data_num_imu=round(size(data_imu,1));
j=0;
timestamp_imu = zeros(data_num_imu,1);
IMU = zeros(data_num_imu,3);
AHRS = zeros(data_num_imu,1);
GPS_VEL = zeros(data_num_imu,4);
Quat = zeros(data_num_imu,4);
GPS_POS = zeros(data_num_imu,3);
for i=1:data_num_imu
    j=j+1;
    timestamp_imu(j,1) = data_imu(i,1);
    IMU(j,1) = data_imu(i,2);
    IMU(j,2) = data_imu(i,3);
    IMU(j,3) = data_imu(i,4);
    AHRS(j,1) = data_imu(i,5);
    GPS_VEL(j,1) = data_imu(i,6);
    GPS_VEL(j,2) = data_imu(i,7);
    GPS_VEL(j,3) = data_imu(i,8);
    GPS_VEL(j,4) = data_imu(i,9);
    Quat(j,1) = data_imu(i,10);
    Quat(j,2) = data_imu(i,11);
    Quat(j,3) = data_imu(i,12);
    Quat(j,4) = data_imu(i,13);
    GPS_POS(j,1) = data_imu(i,14);
    GPS_POS(j,2) = data_imu(i,15);
    GPS_POS(j,3) = data_imu(i,16);
end

g = 9.80665;

%% 初始化状态
Vel_bias = [0.01; 0.01; 0.01];
%[Vel_N;Vel_E;Vel_D;Vel_bias_N;Vel_bias_E;Vel_bias_D]
X = [0;0;0;Vel_bias];

%协方差
P = eye(6);

%过程噪声
vel_process_noise = 1e-8 * ones(1, 3);
vel_bias_process_noise = 1e-8 * ones(1, 3);
Q = diag([vel_process_noise,vel_bias_process_noise]);

%测量噪声
gps_measure_vel_noise = 0.1 * ones(1, 3);
R = diag(gps_measure_vel_noise);

%% KF速度估计
for k=1:data_num_imu
    if(k>1)
        deltaT = timestamp_imu(k,1)-timestamp_imu(k-1,1);
    else
        deltaT = 0;
    end
    deltaT = deltaT*1e-6;
    
    %得到机体坐标系下XYZ三轴加速度
    aMeas_body_X = IMU(k,1);
    aMeas_body_Y = IMU(k,2);
    aMeas_body_Z = IMU(k,3);
    aMeas_body = [aMeas_body_X;aMeas_body_Y;aMeas_body_Z];
    
    quaternion = [Quat(k,1),Quat(k,2),Quat(k,3),Quat(k,4)];
    Tbn = Quat2Tbn(quaternion);
    
    %得到参考坐标系下的NED三轴加速度
    aMeas = Tbn*aMeas_body;
    aMeas_nav_N = aMeas(1);
    aMeas_nav_E = aMeas(2);
    aMeas_nav_D = aMeas(3);
    
    %得到GNSS观测的NED三轴加速度
    gpsMeas_Gspd = GPS_VEL(k,1);
    gpsMeas_N = GPS_VEL(k,2);
    gpsMeas_E = GPS_VEL(k,3);
    gpsMeas_D = GPS_VEL(k,4);
    gpsMeas_pos_N = GPS_POS(k,1);
    gpsMeas_pos_E = GPS_POS(k,2);
    gpsMeas_pos_D = GPS_POS(k,3);
    
    %% 预测
    %vel_k = vel_k-1 + (Acc - a_bias_k-1)*deltaT
    %a_bias_k = a_bias_k-1
    f11 = eye(3);f12 = -eye(3);
    f21 = zeros(3);f22 = eye(3);
    F = [f11,f12;
         f21,f22,];
    dVel_N = deltaT*aMeas_nav_N;
    dVel_E = deltaT*aMeas_nav_E;
    dVel_D = deltaT*aMeas_nav_D;
    
    Bu = [dVel_N;
          dVel_E;
          dVel_D+g*deltaT;
          0;
          0;
          0;];
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
    Z = [gpsMeas_N;gpsMeas_E;gpsMeas_D];
    Hx = H*X_next;
    S = Z - Hx;
    %disp(S);
   
    %计算卡尔曼增益
    SP = H*P_next*H' + R;
    K = P_next*H'*(SP)^-1;
    
    %状态更新
    X = X_next + K*S;
    %协方差更新
    P = (eye(6) - K*H)*P_next;
    
    %disp(X');
    EKF_GSpeed(k,1) = sqrt(X(1)*X(1)+X(2)*X(2));
    EKF_ZSpeed(k,1) = X(3);
end

figure;
plot(timestamp_imu,GPS_VEL(:,1),timestamp_imu,EKF_GSpeed, timestamp_imu,AHRS(:,1));
%plot(timestamp_imu,GPS_VEL(:,1), timestamp_imu,EKF_GSpeed(:,1));
%plot(timestamp_imu,GPS_VEL(:,1), timestamp_imu,AHRS(:,1));
%plot(timestamp_imu,EKF_GSpeed, timestamp_imu,AHRS(:,1));
legend('GPS','EKF','AHRS','FontSize',10);
xlabel('Time(ms)','FontSize',20);
ylabel('Velocity(m/s)','FontSize',20);
title('Ground Speed','FontSize',20);

% figure;
% plot(timestamp_imu,EKF_ZSpeed,timestamp_imu,GPS(:,4));
% legend('EKF','GPS','FontSize',10);
% xlabel('Time(t/ms)','FontSize',20);
% ylabel('Velocity(m/s)','FontSize',20);
% title('Z Speed','FontSize',20);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%