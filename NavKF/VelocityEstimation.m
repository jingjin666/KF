clc;
clear;
close all;

file_imu = 'JKFS_sitl_200hz.csv';
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

%% ��ʼ��״̬
a_bias = [0.01; 0.01; 0.01];
X = [0;0;0;a_bias];

%Э����
P = zeros(6);
P(1,1) = 0.5;
P(2,2) = 0.5;
P(3,3) = 0.7;
P(4,4) = 0.01;
P(5,5) = 0.01;
P(6,6) = 0.01;

%��������
p_noise = 1e-6;
a_process_noise = p_noise * ones(1, 3);
a_bias_xy_process_noise = p_noise * ones(1, 2);
a_bias_z_process_noise = p_noise * ones(1, 1);
Q = diag([a_process_noise,a_bias_xy_process_noise,a_bias_z_process_noise]);

%��������
gps_xy_measure_noise = 0.05 * ones(1, 2);
gps_z_measure_noise = 0.07 * ones(1, 1);
R = diag([gps_xy_measure_noise,gps_z_measure_noise]);

%% KF�ٶȹ���
for k=1:data_num_imu
    if(k>1)
        deltaT = timestamp_imu(k,1)-timestamp_imu(k-1,1);
    else
        deltaT = 0;
    end
    deltaT = deltaT*1e-6;
    
    %�õ���������ϵ��XYZ������ٶ�
    aMeas_body_X = IMU(k,1);
    aMeas_body_Y = IMU(k,2);
    aMeas_body_Z = IMU(k,3);
    aMeas_body = [aMeas_body_X;aMeas_body_Y;aMeas_body_Z];
    
    quaternion = [Quat(k,1),Quat(k,2),Quat(k,3),Quat(k,4)];
    Tbn = Quat2Tbn(quaternion);
    
    %�õ��ο�����ϵ�µ�NED������ٶ�
    aMeas = Tbn*aMeas_body;
    aMeas_nav_N = aMeas(1);
    aMeas_nav_E = aMeas(2);
    aMeas_nav_D = aMeas(3);
    
    %�õ�GNSS�۲��NED������ٶ�
    gpsMeas_N = GPS_VEL(k,1);
    gpsMeas_E = GPS_VEL(k,2);
    gpsMeas_D = GPS_VEL(k,3);
    
    %% Ԥ��
    %vel_k = vel_k-1 + (Acc - a_bias_k-1)*deltaT
    %a_bias_k = a_bias_k-1
    f1 = -deltaT * eye(3);
    F = [eye(3),f1;
         zeros(3),eye(3)];
%     Bu = [deltaT*aMeas_nav_N;
%           deltaT*aMeas_nav_E;
%           deltaT*(aMeas_nav_D+g);
%           0;
%           0;
%           0];
    X_LAST = [X(1);X(2);X(3)];
    X_LAST_ACC_BIAS = [X(4);X(5);X(6)];
    X_VEL_Truth = aMeas_body - X_LAST_ACC_BIAS;
    X_VEL_Truth = Tbn*X_VEL_Truth;
    X_VEL_Truth = X_VEL_Truth+[0;0;g];
    X_VEL_Truth = X_LAST + X_VEL_Truth*deltaT;
    X_next = [X_VEL_Truth;X_LAST_ACC_BIAS];
    %״̬Ԥ��
%     X_next = F*X + Bu;
    %disp(X_next');
    %Э����Ԥ��
    P_next = F*P*F' + Q;
    
    %% ����
   
    %����в�
    H = [1,0,0,0,0,0;
         0,1,0,0,0,0;
         0,0,1,0,0,0];
    Z = [gpsMeas_N;gpsMeas_E;gpsMeas_D];
    Hx = H*X_next;
    S = Z - Hx;
    %disp(S);
   
    %���㿨��������
    SP = H*P_next*H' + R;
    K = P_next*H'*(SP)^-1;
    
    %״̬����
    X = X_next + K*S;
    
    X(4:6) = Tbn'* X(4:6);
    %Э�������
    P = (eye(6) - K*H)*P_next;
    
    %disp(X');
    JKF_GSpeed(k,1) = sqrt(X(1)*X(1)+X(2)*X(2));
    JKF_Speed(k,1) = X(1);
    JKF_Speed(k,2) = X(2);
    JKF_Speed(k,3) = X(3);
end

figure;
%plot(timestamp_imu,GPS_VEL_XY,timestamp_imu,JKF_GSpeed,timestamp_imu,AHRS_VEL_XY);
plot(timestamp_imu,GPS_VEL_XY, timestamp_imu,JKF_GSpeed);
%plot(timestamp_imu,GPS_VEL_XY, timestamp_imu,AHRS_VEL_XY);
%plot(timestamp_imu,JKF_GSpeed, timestamp_imu,AHRS_VEL_XY);
legend('GPS','JKF','AHRS','FontSize',10);
xlabel('Time(ms)','FontSize',20);
ylabel('Velocity(m/s)','FontSize',20);
title('Ground Speed','FontSize',20);

% figure;
% plot(timestamp_imu,GPS_VEL(:,1),timestamp_imu,JKF_Speed(:,1));
% legend('GPS','JKF','FontSize',10);
% xlabel('Time(ms)','FontSize',20);
% ylabel('Velocity(m/s)','FontSize',20);
% title('X Speed','FontSize',20);
% 
% figure;
% plot(timestamp_imu,GPS_VEL(:,2),timestamp_imu,JKF_Speed(:,2));
% legend('GPS','JKF','FontSize',10);
% xlabel('Time(ms)','FontSize',20);
% ylabel('Velocity(m/s)','FontSize',20);
% title('Y Speed','FontSize',20);
% 
% figure;
% plot(timestamp_imu,GPS_VEL(:,3),timestamp_imu,JKF_Speed(:,3));
% legend('GPS','JKF','FontSize',10);
% xlabel('Time(ms)','FontSize',20);
% ylabel('Velocity(m/s)','FontSize',20);
% title('Z Speed','FontSize',20);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%