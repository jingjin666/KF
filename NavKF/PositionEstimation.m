clc;
clear;
close all;

file_imu = 'JKFS_sitl.csv';
data_list_imu = importdata(file_imu);
imu_time_list = data_list_imu.textdata;
data_imu = data_list_imu.data;

data_num_imu=round(size(data_imu,1));
j=0;
timestamp_imu = zeros(data_num_imu,1);
IMU = zeros(data_num_imu,3);
Quat = zeros(data_num_imu,4);
AHRS_VEL_XY = zeros(data_num_imu,1);
GPS_VEL_XY = zeros(data_num_imu,1);
GPS_VEL = zeros(data_num_imu,3);
GPS_POS = zeros(data_num_imu,3);
for i=1:data_num_imu
    j=j+1;
    timestamp_imu(j,1) = data_imu(i,1);
    IMU(j,1) = data_imu(i,2);
    IMU(j,2) = data_imu(i,3);
    IMU(j,3) = data_imu(i,4);
    Quat(j,1) = data_imu(i,5);
    Quat(j,2) = data_imu(i,6);
    Quat(j,3) = data_imu(i,7);
    Quat(j,4) = data_imu(i,8);
    AHRS_VEL_XY(j,1) = data_imu(i,9);
    GPS_VEL_XY(j,1) = data_imu(i,10);
    GPS_VEL(j,1) = data_imu(i,11);
    GPS_VEL(j,2) = data_imu(i,12);
    GPS_VEL(j,3) = data_imu(i,13);
    GPS_POS(j,1) = data_imu(i,14);
    GPS_POS(j,2) = data_imu(i,15);
    GPS_POS(j,3) = data_imu(i,16);
end

g = 9.80665;

%% 初始化状态
Vel_bias = [0.01; 0.01; 0.01];
%[Vel_N;Vel_E;Vel_D;Pos_N;Pos_E;Pos_D;Vel_bias_N;Vel_bias_E;Vel_bias_D]
X = [0;0;0;30.5601984;104.1952896;584.09;Vel_bias];

%协方差
P = eye(9);

%过程噪声
vel_p_noise = 1e-4;
vel_process_noise = 1e-5 * ones(1, 3);
vel_bias_process_noise = 1e-5 * ones(1, 3);
pos_bias_process_noise = 1e-6 * ones(1, 3);
Q = diag([vel_process_noise,vel_bias_process_noise,pos_bias_process_noise]);

%测量噪声
gps_measure_vel_noise = 0.3 * ones(1, 3);
gps_measure_pos_noise = 0.001 * ones(1, 3);
R = diag([gps_measure_vel_noise,gps_measure_pos_noise]);

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
    gpsMeas_Gspd = GPS_VEL_XY(k,1);
    gpsMeas_N = GPS_VEL(k,1);
    gpsMeas_E = GPS_VEL(k,2);
    gpsMeas_D = GPS_VEL(k,3);
    gpsMeas_pos_N = GPS_POS(k,1);
    gpsMeas_pos_E = GPS_POS(k,2);
    gpsMeas_pos_D = GPS_POS(k,3);
    
    %% 预测
    %vel_k = vel_k-1 + (Acc - a_bias_k-1)*deltaT
    %a_bias_k = a_bias_k-1
    f11 = eye(3);f12 = -eye(3);f13 = zeros(3);
    f21 = zeros(3);f22 = eye(3);f23 = zeros(3);
    f31 = 0.5*deltaT*eye(3);
    f31(1,1) = f31(1,1)/111000;
    f31(2,2) = f31(2,2)/853900;
    f32 = -0.5*deltaT*eye(3);
    f32(1,1) = f32(1,1)/111000;
    f32(2,2) = f32(2,2)/853900;
    f33 = eye(3);
    F = [f11,f12,f13;
         f21,f22,f23;
         f31,f32,f33];
    dVel_N = deltaT*aMeas_nav_N;
    dVel_E = deltaT*aMeas_nav_E;
    dVel_D = deltaT*aMeas_nav_D;
    
    dPos_N = 0.5*dVel_N*deltaT/111000;
    dPos_E = 0.5*dVel_E*deltaT/853900;
    dPos_D = 0.5*dVel_D*deltaT;
    Bu = [dVel_N;
          dVel_E;
          dVel_D+g*deltaT;
          0;
          0;
          0;
          dPos_N;
          dPos_E;
          dPos_D+0.5*g*deltaT*deltaT];
    %状态预测
    X_next = F*X + Bu;

    %disp(X_next');
    %协方差预测
    P_next = F*P*F' + Q;
    
    %% 更新
   
    %计算残差
    H = [1,0,0,0,0,0,0,0,0;
         0,1,0,0,0,0,0,0,0;
         0,0,1,0,0,0,0,0,0;
         0,0,0,0,0,0,1,0,0;
         0,0,0,0,0,0,0,1,0;
         0,0,0,0,0,0,0,0,1];
    Z = [gpsMeas_N;gpsMeas_E;gpsMeas_D;gpsMeas_pos_N;gpsMeas_pos_E;gpsMeas_pos_D];
    Hx = H*X_next;
    S = Z - Hx;
    %disp(S);
   
    %计算卡尔曼增益
    SP = H*P_next*H' + R;
    K = P_next*H'*(SP)^-1;
    
    %状态更新
    X = X_next + K*S;
    %协方差更新
    P = (eye(9) - K*H)*P_next;
    
    %disp(X');
    EKF_GSpeed(k,1) = sqrt(X(1)*X(1)+X(2)*X(2));
    EKF_ZSpeed(k,1) = X(3);
    
    EKF_Pos(k,1) = X(7);
    EKF_Pos(k,2) = X(8);
    EKF_Pos(k,3) = X(9);
end

% figure;
% %plot(timestamp_imu,GPS_VEL_XY(:,1),timestamp_imu,EKF_GSpeed, timestamp_imu,AHRS_VEL_XY(:,1));
% plot(timestamp_imu,GPS_VEL_XY(:,1), timestamp_imu,EKF_GSpeed(:,1));
% %plot(timestamp_imu,GPS_VEL_XY(:,1), timestamp_imu,AHRS_VEL_XY(:,1));
% %plot(timestamp_imu,EKF_GSpeed, timestamp_imu,AHRS_VEL_XY(:,1));
% legend('GPS','EKF','AHRS_VEL_XY','FontSize',10);
% xlabel('Time(ms)','FontSize',20);
% ylabel('Velocity(m/s)','FontSize',20);
% title('Ground Speed','FontSize',20);

% figure;
% plot(timestamp_imu,GPS_POS(:,1),timestamp_imu,EKF_Pos(:,1));
% legend('GPS_POS','EKF_Pos','FontSize',10);
% xlabel('Time(ms)','FontSize',20);
% ylabel('Lat(deg)','FontSize',20);
% title('POS','FontSize',20);

% figure;
% plot(timestamp_imu,GPS_POS(:,2), timestamp_imu,EKF_Pos(:,2));
% legend('GPS_POS','EKF_Pos','FontSize',10);
% xlabel('Time(ms)','FontSize',20);
% ylabel('Lng(deg)','FontSize',20);
% title('POS','FontSize',20);
% 
figure;
plot(timestamp_imu,GPS_POS(:,3), timestamp_imu,EKF_Pos(:,3));
legend('GPS_POS','EKF_Pos','FontSize',10);
xlabel('Time(ms)','FontSize',20);
ylabel('Alt(m)','FontSize',20);
title('POS','FontSize',20);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%