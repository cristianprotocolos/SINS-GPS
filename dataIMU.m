% lecturas dataIMU.m
data_imu = readmatrix("C:\Users\cgon0\Desktop\paper odometria\1_exp.csv"); % sinBateriaEsTest1
gps_true_plaza = readmatrix("C:\Users\cgon0\Desktop\paper odometria\gps_true_plaza.csv");
gps_med = data_imu(:,23:24);

lat_NZ = nonzeros(gps_med(:,1));
log_NZ = nonzeros(gps_med(:,2));

gps_med_NZ = [lat_NZ log_NZ];

seleccionados = [(1:10000)]; %%cambios ={1048, 1891, 2806, 3793
quat = data_imu(:,10:13); %bno acce 12 a 15  quat 22, 22,23,25
q0 = quat(:,1);
q1 = quat(:,2);
q2 = quat(:,3);
q3 = quat(:,4);

w_body = data_imu(:,7:9); %Velocidad angular bno 7 a 9
Scaling_g = 0.061 * pi / 180;
w_x = w_body(:,1) * Scaling_g;
w_y = w_body(:,2) * Scaling_g;
w_z = w_body(:,3) * Scaling_g;


%% matrices de calibracion del acelerometro
am = data_imu(seleccionados,14:16); %% aceleraciones mpu
%am = data_imu(seleccionados, 1: 3); %% aceleraciones bno