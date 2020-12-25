clc;
clear;
close all;

filename = 'session.txt';
data = load(filename);

data_num=round(size(data,1));
j=0;
for i=1:data_num
    j=j+1;
    DATA_CNTR(j,1) = i;
    roll_ekf(j,1) = mean(data(i,1));
    roll_acc(j,1) = mean(data(i,2));
    pitch_ekf(j,1) = mean(data(i,3));
    pitch_acc(j,1) = mean(data(i,4));
    yaw_ekf(j,1) = mean(data(i,5));
    yaw_mag(j,1) = mean(data(i,6));
end

Time = 1:1:data_num;
Time = Time';

figure;
plot(Time,roll_acc,Time,roll_ekf);
legend('roll-acc','roll-ekf','FontSize',10);
xlabel('t / s','FontSize',20);
ylabel('roll','FontSize',20);
title('roll','FontSize',20);

figure;
plot(Time,pitch_acc,Time,pitch_ekf);
legend('pitch-acc','pitch-ekf','FontSize',10);
xlabel('t / s','FontSize',20);
ylabel('pitch','FontSize',20);
title('pitch','FontSize',20);

figure;
plot(Time,yaw_mag,Time,yaw_ekf);
legend('yaw-mag','yaw-ekf','FontSize',10);
xlabel('t / s','FontSize',20);
ylabel('yaw','FontSize',20);
title('yaw','FontSize',20);

disp("ekf filter");