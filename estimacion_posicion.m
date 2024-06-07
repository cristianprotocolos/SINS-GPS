clear
clc
close all

%% Optimizacion
%% Modelo
bias = [-0.00703865554468412	0.119189157604515 0];
%bias_op = fminsearch(@recorridoReal, bias)
%bias_op = [-0.00703865554468412	0.119189157604515];
% bias_x = bias_op(1);
% bias_y = bias_op(2);

%% Funcion objetivo
%function J = recorridoReal(bias);

bias_x = bias(1)*0;
bias_y = bias(2)*0;
theta_bias  = bias(3)*0;
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

% am_BNO = data_imu(seleccionados,1:3);
% a_x = am_BNO(:,1)*0.0010;
% a_y = am_BNO(:,2)*0.0010;
% a_z = am_BNO(:,3)*0.0010;
%% rotación de coordenadas de aceleración
for k=1:N
    scaling = norm([q0(k) q1(k) q2(k) q3(k)]);

    q0_k = q0(k)/scaling;
    q1_k = q1(k)/scaling;
    q2_k = q2(k)/scaling;
    q3_k = q3(k)/scaling;

    theta = theta_bias; % Ángulo de rotación en grados
    theta_rad = deg2rad(theta);
    
    % Crear el cuaternión de rotación en el eje Z
    qz = [cos(theta_rad / 2), 0, 0, sin(theta_rad / 2)];
    
    % Definir el cuaternión original
    q_original = [q0_k, q1_k, q2_k, q3_k];
    
    % Multiplicar el cuaternión original por el cuaternión de rotación
    q_rot = quatmultiply(qz, q_original);

    q0_k = q_rot(1);
    q1_k = q_rot(2);
    q2_k = q_rot(3);
    q3_k = q_rot(4);

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

acc_Wx = ( acc_Wx + bias_x ); % se agragan bias a las aceleraciones 
acc_Wy = ( acc_Wy + bias_y );

% w_norm = zeros(1,N);
% contCorrec = 0;
% inicio = 301;
% Kcorrec = 1;

%% TIEMPOS
S_IMU = [Sx_k; Sy_k; Sz_k]';
med_m = [Sx_k; Sy_k]';

[dist_true, desp_true, coord_XY] = gps_med2m(gps_med_NZ);
[dist_med, desp_med] = rec_desp(med_m);

N_imu = length(desp_med);
N_gps = length(desp_true);

tiempo_imu = linspace(1, seleccionados(end)/10, N_imu);
tiempo_gps = linspace(1, seleccionados(end)/10, N_gps);

indices_no_cero = find(gps_med(:,1) ~= 0);
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

k_z = 1;

% for k=inicio:N-1
%     a_norm(k) = sqrt(acc_Wx(k)^2 + acc_Wy(k)^2 + acc_Wz(k)^2); 
%     w_norm(k) = sqrt(w_x(k)^2 + w_y(k)^2 + w_z(k)^2);     
% end


% ref_acc = mean(acc_Wz(end-100:end));
% delta_Ref_acc = std(acc_Wz(end-100:end));
% ref_w = mean(w_norm(end-100:end));
% delta_Ref_w = std(w_norm(end-100:end));
inicio = 2;
for k=inicio:N-1
%     w_norm(k) = sqrt(w_x(k)^2 + w_y(k)^2 + w_z(k)^2); 
%     a_norm(k) = sqrt(acc_Wx(k)^2 + acc_Wy(k)^2 + acc_Wz(k)^2); 
% 
%     if  (ref_w-delta_Ref_w  < w_norm(k) && w_norm(k) < ref_w+delta_Ref_w) 
%        % || (ref_acc-delta_Ref_acc  < acc_Wz(k) && acc_Wz(k) < ref_acc+delta_Ref_acc)
% 
%         a=0;b=0;c=0; 
%         contCorrec = contCorrec + 1;
%     else
%         a=1;b=1;c=1;
%     end

%     if mod(k, 90) == 0
%         a=0;b=0;c=0;
%     else
%         a=1;b=1;c=1;
%     end
    
    % Body Frame
%     Vx_body(k) = Vx_body(k-1)*a + dt * a_x(k)*b;
%     Vy_body(k) = Vy_body(k-1)*a + dt * a_y(k)*c;
%     Vz_body(k) = Vz_body(k-1)*a + dt * a_z(k);
    
    % World Frame
    Vx_k(k) = Vx_k(k-1)*1 + dt * acc_Wx(k)*1;
    Vy_k(k) = Vy_k(k-1)*1 + dt * acc_Wy(k)*1;
    Vz_k(k) = Vz_k(k-1)*1 + dt * acc_Wz(k);
    
   
    th_x = 1.3000000;
    th_y = 1.30000;

    if abs(Vx_k(k)) > th_x
        Vx_k(k) = Vx_k(k-1);
    end
    
    if abs(Vy_k(k)) > th_y
        Vy_k(k) = Vy_k(k-1);
    end

    % Body Frame
%     Sx_body(k) = Sx_body(k-1) + dt * Vx_body(k);
%     Sy_body(k) = Sy_body(k-1) + dt * Vy_body(k);
%     Sz_body(k) = Sz_body(k-1) + dt * Vz_body(k);
    
    % World Frame
    Sx_k(k) = Sx_k(k-1) + dt * Vx_k(k);
    Sy_k(k) = Sy_k(k-1) + dt * Vy_k(k);
    Sz_k(k) = Sz_k(k-1) + dt * Vz_k(k);
   
    %Sx_k(k) = (sqrt(Sx_k(k)^2+Sy_k(k)^2))*cos(theta);
    %Sy_k(k) = (sqrt(Sx_k(k)^2+Sy_k(k)^2))*sin(theta);
    %% FILTRO DE KALMAN EXTENDIDO

    % Predicción
%     x_pred = [Sx_k(k); Sy_k(k); Vx_k(k); Vy_k(k)];  % Estado predicho
%     Fk = F;  % Evaluar la matriz Jacobiana en el estado actual
%     P = Fk * P * Fk' + Q;  % Covarianza predicha

    % Actualización (cada 10 segundos)

    
%     if gps_med(k,1) ~= 0 && ( mod(k_z, 422220) == 0 || k_z == 1 ) && correcOn
%         
%         % Obtener una nueva medición (por ejemplo, desde un GPS)
%         z = [coord_XY(k_z,1); coord_XY(k_z,2)];
% 
%         % Calcular la ganancia de Kalman
%         K = P * H' / (H * P * H' + R);
% 
%         % Actualizar la estimación del estado
%         x_h = x_pred + K * (z - H * x_pred);
% 
%         % Actualizar la covarianza del error
%         P = (eye(size(K,1)) - K * H) * P;
%         tiempos_correc(Kcorrec) = k;
%         Kcorrec = 1 + Kcorrec;
%         %correcOn = 0;
% 
%     else
%         % Sin medición, estado predicho se convierte en el estado actual
%         x_h = x_pred;
%     end
    
%     if gps_med(k,1) ~= 0
%         k_z = 1 + k_z;
%     end
    
%     Sx_k(k) = x_h(1);
%     Sy_k(k) = x_h(2);
% 
%     Vx_k(k) = x_h(3);
%     Vy_k(k) = x_h(4);

%     if abs(Vx_k(k)) > th_x
%         Vx_k(k) = Vx_k(k-1);
%     end
%     
%     if abs(Vy_k(k)) > th_y
%         Vy_k(k) = Vy_k(k-1);
%     end

%     % calculo de angulos
%     u = Sx_k(k);
%     v = Sy_k(k);
%     theta_r = atan2(v,u);
%     %theta_d(k) = rad2deg(theta_r);
%     theta_d(k) = theta_r;
% 
%     theta_error = 0.0102;
%     s_norm = norm([u, v]);
% 
%     Sx_k(k) = s_norm*cos(theta_d(k) + theta_error);
%     Sy_k(k) = s_norm*sin(theta_d(k) + theta_error);


%% RMSE

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


%     J = sqrt( sum(Sx_k - X).^2 + sum(Sy_k - Y).^2 );
%     dif_x = abs(Sx_k - X); max_dif_x = max(dif_x)
%     dif_y = abs(Sy_k - Y); max_dif_y = max(dif_y)
%     J = J + abs(max_dif_x) + abs(max_dif_y)


%end


%% calculo de angulos de mediciones GPS

% for i=1:length(coord_XY((1:end),1))
%     ref = [coord_XY(1,1) coord_XY(1,2)];
%     angleGPS(i) = atan2(coord_XY(i,1), coord_XY(i,2)); 
% end

end

j =  1;
R = [];
for i=1:indices_no_cero(end)
    if indices_no_cero(j) == i
        R(j) = (Sx_k(i) - gps_med_m(i,1)).^2 + (Sy_k(i) - gps_med_m(i,2)).^2;
        j=j+1;
    end     
end

J = sqrt(sum(R))

%end

%k=0.77;
% az_max=0.37;
% az_min=-0.32;
% L = k*(az_max - az_min)^(1/4)

% FIGURAS
onFig = 1;
if onFig == 1

    inicio = 301;
    x_min = 5016;
    x_max = x_min + 10*5;
    y_min = -4;
    y_max = 4;
    
    figure(1)
    subplot(1,3,1), plot(acc_Wx),legend("acc_Wx"); axis([x_min x_max -1 1])
    subplot(1,3,2), plot(acc_Wy),legend("acc_Wy"); axis([x_min x_max -1 1])
    subplot(1,3,3), plot(acc_Wz),legend("acc_Wz"); axis([x_min x_max -1 1])

    
    figure(2)
    subplot(3,1,1), plot(a_x),legend("acc_Bx"); axis([x_min x_max 0 2])
    subplot(3,1,2), plot(a_y),legend("acc_By"); axis([x_min x_max -1 1])
    subplot(3,1,3), plot(a_z),legend("acc_Bz"); axis([x_min x_max -1 1])
    
    figure(3)
    subplot(3,1,1), plot(Vx_k),legend("V_x")
    subplot(3,1,2), plot(Vy_k),legend("V_y")
    subplot(3,1,3), plot(Vz_k),legend("V_z")
    
    figure(4)
    subplot(3,1,1), plot(Sx_k),legend("S_x")
    subplot(3,1,2), plot(Sy_k),legend("S_y")
    subplot(3,1,3), plot(Sz_k),legend("S_z")
    
    figure(5)
    hold on
    fin=10000;
    plot(coord_XY((1:end),1), coord_XY((1:end),2),':', 'LineWidth', 2)
    plot(Sx_k(1:fin-1), Sy_k(1:fin-1),':', 'LineWidth', 2) % frame WORLD  
    %plot(Sx_body, Sy_body,':', 'LineWidth', 2) % frame BODY
    legend("TRUE", "IMU_W", "IMU_B", 'Location', 'best')
    
    figure(6)
    subplot(2,1,1), plot(tiempo_gps,  coord_XY(1:end-1,1))
    hold on
    plot(tiempo_imu, Sx_k(1:end-1))
    hold off
    legend("TRUE", "IMU", 'Location', 'best')
    
    subplot(2,1,2), plot(tiempo_gps,  coord_XY(1:end-1,2))
    hold on
    plot(tiempo_imu, Sy_k(1:end-1))
    hold off
    legend("TRUE", "IMU", 'Location', 'best')
    
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
    subplot(1,4,1), plot(quat(inicio:N,1)/scaling), axis([inicio N -1 1]); axis([x_min x_max -1 1])
    subplot(1,4,2), plot(quat(inicio:N,2)/scaling), axis([inicio N -1 1]); axis([x_min x_max -1 1])
    subplot(1,4,3), plot(quat(inicio:N,3)/scaling), axis([inicio N -1 1]); axis([x_min x_max -1 1])
    subplot(1,4,4), plot(quat(inicio:N,4)/scaling), axis([inicio N -1 1]); axis([x_min x_max -1 1])
    
    % figure(11)
    % subplot(3,1,1), plot(am_BNO(:,1)),legend("acc_Bx")
    % subplot(3,1,2), plot(am_BNO(:,2)),legend("acc_By")
    % subplot(3,1,3), plot(am_BNO(:,3)),legend("acc_Bz")
    
    figure(12)
    title('Velocidad Angular BNO055')
    subplot(1,3,1), plot(w_x),legend("Giro_x"); axis([x_min x_max y_min y_max])
    subplot(1,3,2), plot(w_y),legend("Giro_y"); axis([x_min x_max y_min y_max])
    subplot(1,3,3), plot(w_z),legend("Giro_z"); axis([x_min x_max y_min y_max])
    
    figure(13)
    plot(rad2deg(angleGPS))
end
% 
