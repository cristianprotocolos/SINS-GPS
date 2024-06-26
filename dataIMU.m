% dataIMU.m
dir = pwd;
data = readmatrix([dir, '\datos\testIsC.csv']);
nData = 5000:8000;

data_imu = data(:, 1:22);
% data_gps = readmatrix([dir, '\datos\gps_med_exp1.csv']);
data_gps = data(:, 23:24);

true_gps_exp1 = readmatrix([dir, '\datos\true_gps_exp1.csv']);

lat_NZ = nonzeros(data_gps(:,1));
log_NZ = nonzeros(data_gps(:,2));

for i=1:length(data_gps(:,1))
    if (data_gps(i,1) ~= 0)
        gps_disp(i) = 1;
    else
        gps_disp(i) = 0;
    end
end

gps_med_NZ = [lat_NZ log_NZ];

%% Cuaterniones BNO055
quat = data_imu(:,10:13); %bno acce 12 a 15  quat 22, 22,23,25
q0_bno = quat(:,1);
q1_bno = quat(:,2);
q2_bno = quat(:,3);
q3_bno = quat(:,4);

for i=1:length(q0_bno)
    kq_bno = norm(quat(i,:));
    qc_bno(i,:) = quat(i,:) / kq_bno;
end

%% Velocidad angular bno 7 a 9
w_body = data_imu(:,7:9); 
Scaling_g = 0.061 * pi / 180;
w_x = w_body(:,1) * Scaling_g;
w_y = w_body(:,2) * Scaling_g;
w_z = w_body(:,3) * Scaling_g;

W = [w_x w_y w_z];
%% Aceleraciones lineales mpu 14 a 16
am = data_imu(:,14:16); %% aceleraciones mpu
%am = data_imu(seleccionados, 1: 3); %% aceleraciones bno
bm = data_imu(:,20:22); %% Magnetometro  mpu 17 a 19

