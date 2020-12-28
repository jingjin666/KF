clc;
clear;
close all;

file_imu = 'imu_nav_sitl.csv';
data_list_imu = importdata(file_imu);
imu_time_list = data_list_imu.textdata;
data_imu = data_list_imu.data;

data_num_imu=round(size(data_imu,1));
j=0;
timestamp_imu = zeros(data_num_imu,1);
IMU = zeros(data_num_imu,3);
AHRS = zeros(data_num_imu,1);
GPS = zeros(data_num_imu,4);
Quat = zeros(data_num_imu,4);
for i=1:data_num_imu
    j=j+1;
    timestamp_imu(j,1) = data_imu(i,1);
    IMU(j,1) = data_imu(i,5);
    IMU(j,2) = data_imu(i,6);
    IMU(j,3) = data_imu(i,7);
    AHRS(j,1) = data_imu(i,8);
    GPS(j,1) = data_imu(i,9);
    GPS(j,2) = data_imu(i,10);
    GPS(j,3) = data_imu(i,11);
    GPS(j,4) = data_imu(i,12);
    Quat(j,1) = data_imu(i,13);
    Quat(j,2) = data_imu(i,14);
    Quat(j,3) = data_imu(i,15);
    Quat(j,4) = data_imu(i,16);
end

g = 9.80665;

%% ��ʼ��״̬
a_bias = [0.01; 0.01; 0.01];
X = [0;0;0;a_bias];

%Э����
P = eye(6);

%��������
p_noise = 1e-4;
a_process_noise = p_noise * ones(1, 3);
a_bias_process_noise = p_noise * ones(1, 3);
Q = diag([a_process_noise,a_bias_process_noise]);

%��������
gps_measure_noise = 0.1 * ones(1, 3);
R = diag(gps_measure_noise);

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
    
    %quaternion = inverse(Quat(k,:))
    quaternion = [Quat(k,1),-Quat(k,2),-Quat(k,3),-Quat(k,4)];
    Tbn = Quat2Tbn(quaternion);
    
    %�õ��ο�����ϵ�µ�NED������ٶ�
    aMeas = Tbn*aMeas_body;
    aMeas_nav_N = aMeas(1);
    aMeas_nav_E = aMeas(2);
    aMeas_nav_D = aMeas(3);
    
    %�õ�GNSS�۲��NED������ٶ�
    gpsMeas_Gspd = GPS(k,1);
    gpsMeas_N = GPS(k,2);
    gpsMeas_E = GPS(k,3);
    gpsMeas_D = GPS(k,4);
    
    %% Ԥ��
    %vel_k = vel_k-1 + (Acc - a_bias_k-1)*deltaT
    %a_bias_k = a_bias_k-1
    f1 = -deltaT * eye(3);
    F = [eye(3),f1;
         zeros(3),eye(3)];
    Bu = [deltaT*aMeas_nav_N;
          deltaT*aMeas_nav_E;
          deltaT*(aMeas_nav_D+g);
          0;
          0;
          0];
    %״̬Ԥ��
    X_next = F*X + Bu;
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
    %Э�������
    P = (eye(6) - K*H)*P_next;
    
    %disp(X');
    EKF_GSpeed(k,1) = sqrt(X(1)*X(1)+X(2)*X(2));
    EKF_ZSpeed(k,1) = X(3);
end

figure;
plot(timestamp_imu,GPS(:,1),timestamp_imu,EKF_GSpeed, timestamp_imu,AHRS(:,1));
%plot(timestamp_imu,GPS(:,1), timestamp_imu,EKF_GSpeed(:,1));
%plot(timestamp_imu,GPS(:,1), timestamp_imu,AHRS(:,1));
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