clear
clc
close all

dataIMU; % Lectura de DATA SD 
constantes; % Lectura de constantes

%% Calibracion de aceleraciones body MPU9250
for i=1:N
    a_b = S_a * k_a * (am(i,:)' - bias_a); %% Body Frame
    a_x(i) = a_b(1)*1;
    a_y(i) = a_b(2)*1;
    a_z(i) = a_b(3)*1;
    norm_acc(i) = norm(a_b);
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
    
    scaling_q = norm([q0(k) q1(k) q2(k) q3(k)]);

    q0_med = q0(k)/scaling_q;
    q1_med = q1(k)/scaling_q;
    q2_med = q2(k)/scaling_q;
    q3_med = q3(k)/scaling_q;
    
    q_k = [q0_med q1_med q2_med q3_med];

    % Angulos de Euler
    pitch_med(k) = atan2( 2*(q0_med*q1_med + q2_med*q3_med), 1 - 2*(q1_med^2 + q2_med^2));
    roll_med(k)  = asin(2*(q0_med*q2_med - q3_med*q1_med));
    yaw_med(k)   = atan2( 2*(q0_med*q3_med + q1_med*q2_med), 1 - 2*(q2_med^2 + q3_med^2));

    % Matriz de rotacion de Body a World
    acc_b = [a_x(k) a_y(k) a_z(k)]';

    q_aceleracion = quaternion(q_k) * acc_b;    
    q_conjugado = conj(quaternion(q_k));   
    acc_W = parts(q_aceleracion * q_conjugado);
   
    % Vectores de aceleraciones calibradas Frame WORLD
    acc_Wx(k) = acc_W(1)*g_w;
    acc_Wy(k) = acc_W(2)*g_w;
    acc_Wz(k) = acc_W(3)*g_w;    

    % Modifica la orientacion de los ejes y elimina componente de gravedad del eje Zw
    aw = [-acc_Wx(k); -acc_Wy(k); acc_Wz(k)]' - [ 0 0 g_w]; %
    aw_k = [-acc_Wx(k); -acc_Wy(k)];

    %% CONDICIONES
    a_norm(k) = sqrt(acc_Wx(k)^2 + acc_Wy(k)^2 + acc_Wz(k)^2); 
    w_norm(k) = sqrt(w_x(k)^2 + w_y(k)^2 + w_z(k)^2); 
    %% condicion 1: thrhd_amin = 1.1; thrhd_amax = 0.9;    
    if ( thrhd_amin < a_norm(k) && thrhd_amax > a_norm(k) )
        C_1(k) = 1;
    else
        C_1(k) = 0;
    end
    %% condicion 2: 
    % Calcular la suma de los valores dentro de la ventana de promedio
    lower_limit = max(k - s, 1);
    upper_limit = min(k + s, numel(aw));
    sum_within_window = sum(aw(lower_limit:upper_limit));
    
    % Calcular el promedio de los valores dentro de la ventana de promedio
    mean_within_window = sum_within_window / window_size;
    
    % Calcular la diferencia entre a_k y el cuadrado del promedio dentro de la ventana de promedio
    diff_square_mean = (aw(3) - mean_within_window)^2;
    
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
    %% FILTRO DE KALMAN EXTENDIDO
    
    % Prediccion
    A_k = [I I*Ts O ; O I*Cs O ; 0 0 0 0 0 0];
    A_k(:,end) = [];
    B_k = [Ts^2/2 * I ; Ts * I ; 0 0];
    O_k = [0 0 0 0 yaw_med(k)]';
    
    x_pred = A_k * x_h + B_k * aw_k + O_k;
    
    H = [I O O ; 0 0 0 0 1 0];
    H(:,end) = [];
    
%     z_pred = H * x_pred;

    % Actualizacion

    %m_k = [x_gps , y_gps , yaw_gps]
end


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