%% En este script simplificacmos los algoritmos del modelos y ajustamos sus hiperparametros para
%% encontrar una configuracion que satisface nuestras necesidades
clear
clc
close all

%% Lectura de datos y valores constante
dataIMU; % Lectura de DATA SD 
constantes; 

%% Calibracion de aceleraciones MPU9250
for i=1:N
    a_b = S_a * k_a * (am(i,:)' - bias_a);
    a_x(i) = a_b(1)*1;
    a_y(i) = a_b(2)*1;
    a_z(i) = a_b(3)*1;
    magACE(i) = norm(a_b);
end

%% rotación de coordenadas de aceleración
for k=1:N
    scaling = norm([q0_bno(k) q1_bno(k) q2_bno(k) q3_bno(k)]);

    q0_k = q0_bno(k)/scaling;
    q1_k = q1_bno(k)/scaling;
    q2_k = q2_bno(k)/scaling;
    q3_k = q3_bno(k)/scaling;

    wR_b = [q0_k^2 + q1_k^2 - q2_k^2 - q3_k^2, 2*(q1_k*q2_k - q0_k*q3_k), 2*(q1_k*q3_k + q0_k*q2_k);
            2*(q1_k*q2_k + q0_k*q3_k), q0_k^2 - q1_k^2 + q2_k^2 - q3_k^2, 2*(q2_k*q3_k - q0_k*q1_k);
           2*(q1_k*q3_k - q0_k*q2_k), 2*(q2_k*q3_k + q0_k*q1_k), q0_k^2 - q1_k^2 - q2_k^2 + q3_k^2];
    
    acc_b = [a_x(k) a_y(k) a_z(k)]';
    acc_W = wR_b * acc_b;
     
    % Vectores de aceleraciones calibradas (WORLD)
    acc_Wx(k) = acc_W(1)*g_w;
    acc_Wy(k) = acc_W(2)*g_w;
    acc_Wz(k) = acc_W(3)*g_w;    

end    

acc_W = [-acc_Wx; -acc_Wy; acc_Wz];

% Elimina componente de gravedad del eje Zw
acc_Wz = acc_Wz + g_w;

%% calculo distancia, desplazamiento y coordenadas gps en unidades (m)
[dist_med, desp_med, coord_XY_med] = gps_med2m(gps_med_NZ);
[dist_true, desp_true, coord_XY_true] = gps_med2m(true_gps_exp1);

indices_no_cero = find(data_gps(:,1) ~= 0);
gps_med_m = [];
j =  1;
for i = 1:indices_no_cero(end)
    if indices_no_cero(j) == i
        gps_med_m(i,1) = coord_XY_med(j,1); % Guardar el valor si es distinto de cero
        gps_med_m(i,2) = coord_XY_med(j,2); % Guardar el valor si es distinto de cero
        j = j + 1;
    else
        gps_med_m(i,1) = 0; % Guardar el valor si es distinto de cero
        gps_med_m(i,2) = 0; % Guardar el valor si es distinto de cero
    end
end

for i = indices_no_cero(end)+1:N
    gps_med_m(i,1) = 0; % Guardar el valor si es distinto de cero
    gps_med_m(i,2) = 0; % Guardar el valor si es distinto de cero
end


inicio = 2;
for k=inicio:N-1   
    % World Frame
    Vx_k(k) = Vx_k(k-1) + dt * acc_Wx(k);
    Vy_k(k) = Vy_k(k-1) + dt * acc_Wy(k);
    Vz_k(k) = Vz_k(k-1) + dt * acc_Wz(k);
    
    th_x = 1.3000;
    th_y = 1.3000;

    if abs(Vx_k(k)) > th_x
        Vx_k(k) = Vx_k(k-1);
    end
    
    if abs(Vy_k(k)) > th_y
        Vy_k(k) = Vy_k(k-1);
    end
    
    % World Frame
    Sx_k(k) = Sx_k(k-1) + dt * Vx_k(k);
    Sy_k(k) = Sy_k(k-1) + dt * Vy_k(k);
    Sz_k(k) = Sz_k(k-1) + dt * Vz_k(k);
   

end


% FIGURAS
onFig = 1;
if onFig == 1
   
    figure
    subplot(1,3,1), plot(acc_Wx),legend("acc_Wx")
    subplot(1,3,2), plot(acc_Wy),legend("acc_Wy")
    subplot(1,3,3), plot(acc_Wz),legend("acc_Wz")
    title('Aceleraciones frame-world')
   
    figure
    subplot(3,1,1), plot(Vx_k),legend("V_x")
    subplot(3,1,2), plot(Vy_k),legend("V_y")
    subplot(3,1,3), plot(Vz_k),legend("V_z")
    title('velocidades frame-world')
       
    figure
    plot(Sx_k(2000:end),Sy_k(2000:end), "b")
    hold on
    plot(coord_XY_med(:,1), coord_XY_med(:,2), "black") 
    %plot(coord_XY_true(:,1), coord_XY_true(:,2), "r") 

    title("Posición MR-EKF")
    legend("MPU9250" , "GPS", "True")
    hold off
    
    figure
    subplot(2,1,1), plot(tiempo_gps,  coord_XY_med(:,1))
    hold on
    plot(tiempo_imu, Sx_k(:))
    hold off
    legend("Med", "IMU", 'Location', 'best')
    
    subplot(2,1,2), plot(tiempo_gps,  coord_XY_med(:,2))
    hold on
    plot(tiempo_imu, Sy_k(:))
    hold off
    legend("Med", "IMU", 'Location', 'best')
    title('Posiciones frame-world')
    
end
% 

