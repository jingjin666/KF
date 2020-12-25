clc;
clear;
close all;

file_ahrs = 'ahrs.csv';
data_list_ahrs = importdata(file_ahrs);
ahrs_time_list = data_list_ahrs.textdata;
data_ahrs = data_list_ahrs.data;

data_num_ahrs=round(size(data_ahrs,1));
j=0;
for i=1:data_num_ahrs
    j=j+1;
    timestamp_ahrs(j,1) = str2timestamp(ahrs_time_list(i, 2));
    AHRS_gSpeed(j,1) = mean(data_ahrs(i,8));
end

%Time_ahrs = 1:1:data_num_ahrs;
%Time_ahrs = Time_ahrs';

% figure;
% plot(timestamp_ahrs,AHRS_gSpeed);
% legend('AHRS','AHRS','FontSize',10);
% xlabel('t/s','FontSize',20);
% ylabel('m/s','FontSize',20);
% title('gSpeed-AHRS','FontSize',20);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

file_gps = 'gps.csv';
data_list_gps = importdata(file_gps);
data_gps = data_list_gps.data;
gps_time_list = data_list_gps.textdata;
data_num_gps=round(size(data_gps,1));

j = 0;
for i=1:data_num_gps
    j=j+1;
    timestamp_gps(j,1) = str2timestamp(gps_time_list(i, 2));
    GPS_gSpeed(j,1) = data_gps(i,10);
end

% Time_gps = 1:1:data_num_gps;
% Time_gps = Time_gps'; 

figure;
plot(timestamp_ahrs,AHRS_gSpeed, timestamp_gps,GPS_gSpeed);
legend('AHRS','GPS','FontSize',10);
xlabel('Time(t/ms)','FontSize',20);
ylabel('Velocity(m/s)','FontSize',20);
title('Ground Speed','FontSize',20);
