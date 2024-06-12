% lecturas dataIMU.m
dir = pwd;
data_imu = readmatrix([dir, '\datos\1_exp.csv']);
gps_med_exp1 = readmatrix([dir, '\datos\gps_med_exp1.csv']);

lat_NZ = nonzeros(gps_med_exp1(:,1));
log_NZ = nonzeros(gps_med_exp1(:,2));

gps_med_NZ = [lat_NZ log_NZ];

%% Cuaterniones BNO055
quat = data_imu(:,10:13); %bno acce 12 a 15  quat 22, 22,23,25
q0 = quat(:,1);
q1 = quat(:,2);
q2 = quat(:,3);
q3 = quat(:,4);

%% Velocidad angular bno 7 a 9
w_body = data_imu(:,7:9); 
Scaling_g = 0.061 * pi / 180;
w_x = w_body(:,1) * Scaling_g;
w_y = w_body(:,2) * Scaling_g;
w_z = w_body(:,3) * Scaling_g;

%% Aceleraciones lineales mpu 14 a 16
am = data_imu(:,14:16); %% aceleraciones mpu
%am = data_imu(seleccionados, 1: 3); %% aceleraciones bno
