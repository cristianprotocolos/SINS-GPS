%% Script estimacion de posicion
clear
clc
close all

%% Lectura de datos y valores constante
dataIMU; % Lectura de DATA SD 
constantes; % Lectura de constantes

%% Calibracion de Aceleraciones y Magnetometro body MPU9250
for i=1:N
    % Acelerometro
    a_b = S_a * k_a * (am(i,:)' - bias_a); %% Body Frame
    a_x(i) = a_b(1)*1;
    a_y(i) = a_b(2)*1;
    a_z(i) = a_b(3)*1;
    norm_acc(i) = norm(a_b);

    % Magnetometro
    mag_b = k_mag * (bm(i,:)' - bias_mag); %% Body Frame
    mag_x(i) = mag_b(1);
    mag_y(i) = mag_b(2);
    mag_z(i) = mag_b(3);
    norm_mag(i) = norm(mag_b);
end

% am_BNO = data_imu(seleccionados,1:3);
% a_x = am_BNO(:,1)*0.0010;
% a_y = am_BNO(:,2)*0.0010;
% a_z = am_BNO(:,3)*0.0010;

%% calculo distancia, desplazamiento y coordenadas gps en unidades (m)
[dist_true, desp_true, coord_XY] = gps_med2m(gps_med_NZ);

indices_no_cero = find(gps_med_exp1(:,1) ~= 0);
gps_med_m = [];
j =  1;
for i = 1:indices_no_cero(end)
    if indices_no_cero(j) == i
        gps_med_m(i,1) = coord_XY(j,1); % Guardar el valor si es distinto de cero
        gps_med_m(i,2) = coord_XY(j,2); % Guardar el valor si es distinto de cero
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

% for k=1:N
%     a_norm(k) = sqrt(acc_Wx(k)^2 + acc_Wy(k)^2 + acc_Wz(k)^2); 
%     w_norm(k) = sqrt(w_x(k)^2 + w_y(k)^2 + w_z(k)^2);     
% end

%% Modelo de predicion de velocidad y posicion

for k=1:N
    
    %% CONDICIONES
    
    %% condicion 1: thrhd_amin = 1.1; thrhd_amax = 0.9;    
    if ( thrhd_amin < a_norm(k) && thrhd_amax > a_norm(k) )
        C_1(k) = 1;
    else
        C_1(k) = 0;
    end
    %% condicion 2: 
    % Calcular la suma de los valores dentro de la ventana de promedio
    lower_limit = max(k - s, 1);
    upper_limit = min(k + s, numel(aw_k));
    sum_within_window = sum(aw_k(lower_limit:upper_limit));
    
    % Calcular el promedio de los valores dentro de la ventana de promedio
    mean_within_window = sum_within_window / window_size;
    
    % Calcular la diferencia entre a_k y el cuadrado del promedio dentro de la ventana de promedio
    diff_square_mean = (aw_k(3) - mean_within_window)^2;
    
    % Calcular la varianza cuadrática media
    
    sigma_sq(k) = diff_square_mean / window_size;
    if ( w_norm(k) < thrhdS )
        C_2(k) = 1;
    else
        C_2(k) = 0;
    end

    %% condicion 3: 
     if ( w_norm(k) < thrhdwmax )
        C_3(k) = 1;
     else
        C_3(k) = 0;
     end

    Cs(k) = C_1(k) * C_2(k) * C_3(k);
    %% Fin de condiciones

    s1(k) = x_h(1);
    s2(k) = x_h(2);

    q0_k(k) = x_h(7);
    q1_k(k) = x_h(8);
    q2_k(k) = x_h(9);
    q3_k(k) = x_h(10);

    bias_x = x_h(11);
    bias_y = x_h(12);
    bias_z = x_h(13);

    % Coreccion de Bias giroscopio
    w_x = (w_body(:,1) + bias_x)* Scaling_g;
    w_y = (w_body(:,2) + bias_y)* Scaling_g;
    w_z = (w_body(:,3) + bias_z)* Scaling_g;

    % rotacion de coordenadas de aceleracion 
    wR_b = [q0_k^2 + q1_k^2 - q2_k^2 - q3_k^2, 2*(q1_k*q2_k - q0_k*q3_k), 2*(q1_k*q3_k + q0_k*q2_k);
            2*(q1_k*q2_k + q0_k*q3_k), q0_k^2 - q1_k^2 + q2_k^2 - q3_k^2, 2*(q2_k*q3_k - q0_k*q1_k);
           2*(q1_k*q3_k - q0_k*q2_k), 2*(q2_k*q3_k + q0_k*q1_k), q0_k^2 - q1_k^2 - q2_k^2 + q3_k^2];
    
    % Angulos de Euler
    pitch_med(k) = atan2( 2*(q0_k*q1_k + q2_k*q3_k), 1 - 2*(q1_k^2 + q2_k^2));
    roll_med(k)  = asin(2*(q0_k*q2_k - q3_k*q1_k));
    yaw_med(k)   = atan2( 2*(q0_k*q3_k + q1_k*q2_k), 1 - 2*(q2_k^2 + q3_k^2));

    % Vectores de aceleraciones calibradas Frame WORLD
    acc_Wx(k) = acc_W(1)*g_w;
    acc_Wy(k) = acc_W(2)*g_w;
    acc_Wz(k) = acc_W(3)*g_w;    

    % Modifica la orientacion de los ejes y elimina componente de gravedad del eje Zw
    acc_b = [a_x(k) a_y(k) a_z(k)]';
    acc_W = wR_b * acc_b;
    aw_k = ([acc_Wx(k); acc_Wy(k); acc_Wz(k)]' + [ 0 0 g_w])'; %

    a_norm(k) = sqrt(acc_Wx(k)^2 + acc_Wy(k)^2 + acc_Wz(k)^2); 
    w_norm(k) = sqrt(w_x(k)^2 + w_y(k)^2 + w_z(k)^2); 
    
    %% INICIO FILTRO DE KALMAN EXTENDIDO
    A_k = [I I*Ts O zeros(3,4); O I*Cs(k) O zeros(3,4); zeros(7,13)];
    B_k = [Ts^2/2 * I O ; Ts * I O];

    S = [-q1_k -q2_k -q3_k ; q0_k -q3_k q2_k ; q3_k q0_k -q1_k ; -q2_k q1_k q0_k];

    C_k = [zeros(6,13) ; zeros(4,6) eye(4,4) -Ts/2*S ; zeros(3,6) zeros(3,4) eye(3,3)];
    D_k = [zeros(4,3) Ts/1 * S ; zeros(3,6)];

    PHI = (A_k + C_k);

    % Prediccion EKF
    x_pred = PHI * x_h + ([B_k ; D_k]) * [aw_k ; W(k,:)']; % x_h = [x1 x2 x3 , v1 v2 v3 , pitch roll yaw]h
    
    C_a = [-q2_k q3_k -q0_k q1_k ; q1_k q0_k q3_k q2_k ; q0_k -q1_k -q2_k q3_k];
    C_m = [q3_k q2_k q1_k q0_k ; q0_k -q1_k  q2_k -q3_k ; -q1_k -q0_k q3_k q2_k];

    P_pred = PHI * P_h * PHI' + Q_k;

    H = [eye(2) zeros(2,11) ; zeros(3,6) C_a O ; zeros(3,6) C_m O]; % dim(8x13) 6x7
    z_pred = H * x_pred;

    % Actualizacion EKF
    m_k = [x_gps y_gps acc_b mag_b]; % x_gps , y_gps, acc(x,y,z), mag(x,y,z)
    K_k = P_pred * H' * inv(H * P_pred * H' + R_k);
    x_h = x_pred + k * (m_k - z_pred);
    P_h = ( eye(13,13) - K_k * H ) * P_pred;
    %% fin del EKF
end

plot(s1,s2)
%% calculo de angulos de mediciones GPS
for i=3:length(coord_XY((1:end),1))
    ref = [0, 1];  
    if i==1
        v1 = [0, 0];
    else
        v1 = [coord_XY(i-1,1), coord_XY(i-1,2)];
    end
    v2 = [coord_XY(i,1), coord_XY(i,2)]; 
    v = v2 - v1;
    % Calcular el producto punto entre el vector v y el eje x
    dot_product = dot(v, ref);
    % Calcular la magnitud de los vectores
    magnitude_v = norm(v);
    magnitude_x_axis = norm(ref);
    % Calcular el ángulo usando la definición del producto punto
    cos_theta = dot_product / (magnitude_v * magnitude_x_axis);
    % Calcular el ángulo en radianes
    theta_rad = acos(cos_theta);
    % Convertir el ángulo a grados
    theta_deg = rad2deg(theta_rad);
    angleGPS(i) = theta_deg;
    
end

% polos = eig(A - LC)

%% FIGURAS
onFig = 0;
if onFig == 1

    inicio = 301;
    x_min = 5016;
    x_max = x_min + 10*5;
    y_min = -4;
    y_max = 4;
%     
    figure(1)
    subplot(1,3,1), plot(acc_Wx),legend("acc_Wx"); axis([x_min x_max -1 1])
    subplot(1,3,2), plot(acc_Wy),legend("acc_Wy"); axis([x_min x_max -1 1])
    subplot(1,3,3), plot(acc_Wz),legend("acc_Wz"); axis([x_min x_max -1 1])
% 
%     figure(2)
%     subplot(3,1,1), plot(a_x),legend("acc_Bx"); axis([x_min x_max 0 2])
%     subplot(3,1,2), plot(a_y),legend("acc_By"); axis([x_min x_max -1 1])
%     subplot(3,1,3), plot(a_z),legend("acc_Bz"); axis([x_min x_max -1 1])
%     
%     figure(3)
%     subplot(3,1,1), plot(Vx_k),legend("V_x")
%     subplot(3,1,2), plot(Vy_k),legend("V_y")
%     subplot(3,1,3), plot(Vz_k),legend("V_z")
%     
%     figure(4)
%     subplot(3,1,1), plot(Sx_k),legend("S_x")
%     subplot(3,1,2), plot(Sy_k),legend("S_y")
%     subplot(3,1,3), plot(Sz_k),legend("S_z")
    
    figure(5)
    hold on
    fin=10000;
    plot(coord_XY((1:end),1), coord_XY((1:end),2),':', 'LineWidth', 2)
    plot(Sx_k(1:fin-1), Sy_k(1:fin-1),':', 'LineWidth', 2) % frame WORLD  
    %plot(Sx_body, Sy_body,':', 'LineWidth', 2) % frame BODY
    legend("TRUE", "IMU_W", "IMU_B", 'Location', 'best')
    
%     figure(6)
%     subplot(2,1,1), plot(tiempo_gps,  coord_XY(1:end-1,1))
%     hold on
%     plot(tiempo_imu, Sx_k(1:end-1))
%     hold off
%     legend("TRUE", "IMU", 'Location', 'best')
%     
%     subplot(2,1,2), plot(tiempo_gps,  coord_XY(1:end-1,2))
%     hold on
%     plot(tiempo_imu, Sy_k(1:end-1))
%     hold off
%     legend("TRUE", "IMU", 'Location', 'best')
    
    % figure(7)
    % subplot(2,1,1), plot(w_norm), legend("w_{norm}")
    % subplot(2,1,2), plot(a_norm), legend("a_{norm}")
    
    % figure(8) 
    % plot(tiempo_gps, dist_true)
    % hold on
    % plot(tiempo_imu, dist_med)
    % legend("TRUE", "IMU")
    % title("RECORRIDO")
    
    % figure(9)
    % plot(tiempo_gps, desp_true)
    % hold on
    % plot(tiempo_imu, desp_med)
    % legend("TRUE", "IMU")
    % title("DESPLAZAMIENTO")
    
    figure(10)
    subplot(1,4,1), plot(quat(inicio:N,1)/scaling_q), axis([inicio N -1 1]); axis([x_min x_max -1 1])
    subplot(1,4,2), plot(quat(inicio:N,2)/scaling_q), axis([inicio N -1 1]); axis([x_min x_max -1 1])
    subplot(1,4,3), plot(quat(inicio:N,3)/scaling_q), axis([inicio N -1 1]); axis([x_min x_max -1 1])
    subplot(1,4,4), plot(quat(inicio:N,4)/scaling_q), axis([inicio N -1 1]); axis([x_min x_max -1 1])

    % figure(11)
    % subplot(3,1,1), plot(am_BNO(:,1)),legend("acc_Bx")
    % subplot(3,1,2), plot(am_BNO(:,2)),legend("acc_By")
    % subplot(3,1,3), plot(am_BNO(:,3)),legend("acc_Bz")
    
    figure(12)
    title('Velocidad Angular BNO055')
    subplot(1,3,1), plot(w_x),legend("Giro_x"); axis([x_min x_max y_min y_max])
    subplot(1,3,2), plot(w_y),legend("Giro_y"); axis([x_min x_max y_min y_max])
    subplot(1,3,3), plot(w_z),legend("Giro_z"); axis([x_min x_max y_min y_max])
    
%     figure(13)
%     subplot(3,1,1), plot(rad2deg(pitch))
%     subplot(3,1,2), plot(rad2deg(roll))
%     subplot(3,1,3), plot(rad2deg(abs(yaw)))
figure(14)
plot(zx_kf, zy_kf)
    
figure(15)
plot(z_angle)   
 
figure(16)
plot(a_k)
N_ak = nnz(a_k)/N * 100
title('N_ak')

figure(17)
subplot(3,1,1),plot(c1_k, 'r')
subplot(3,1,2),plot(c2_k, 'r')
subplot(3,1,3),plot(c1_k, 'r')
hold on
plot(c2_k, 'b')
title('C1 y C2')

figure(18)
plot(angleGPS)

N_c1 = nnz(c1_k)/N * 100
N_c2 = nnz(c2_k)/N * 100

end