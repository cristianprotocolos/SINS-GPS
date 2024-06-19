%% Script estimacion de posicion utilizando un MR-EFK
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
    a_bx(i) = a_b(1)*1;
    a_by(i) = a_b(2)*1;
    a_bz(i) = a_b(3)*1;
    norm_acc(i) = norm(a_b);

    % Magnetometro
    mag_b = k_mag * (bm(i,:)' - bias_mag); %% Body Frame
    mag_bx(i) = mag_b(1);
    mag_by(i) = mag_b(2);
    mag_bz(i) = mag_b(3);
    norm_mag(i) = norm(mag_b);
end

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

%% Modelo de predicion de velocidad y posicion
for k=1:N
     
    s1(k) = x_h(1);
    s2(k) = x_h(2);

    q0_v(k) = x_h(8);
    q1_v(k) = x_h(7);
    q2_v(k) = x_h(10);
    q3_v(k) = x_h(9);

    q0_k = x_h(7);
    q1_k = x_h(8);
    q2_k = x_h(9);
    q3_k = x_h(10);

    bias_x = x_h(11);
    bias_y = x_h(12);
    bias_z = x_h(13);

    % Coreccion de Bias giroscopio
    w_x = (w_body(:,1) + bias_x)* Scaling_g;
    w_y = (w_body(:,2) + bias_y)* Scaling_g;
    w_z = (w_body(:,3) + bias_z)* Scaling_g;

    % Matriz de rotacion b --> W 
    wR_b = [q0_k^2 + q1_k^2 - q2_k^2 - q3_k^2, 2*(q1_k*q2_k - q0_k*q3_k), 2*(q1_k*q3_k + q0_k*q2_k);
            2*(q1_k*q2_k + q0_k*q3_k), q0_k^2 - q1_k^2 + q2_k^2 - q3_k^2, 2*(q2_k*q3_k - q0_k*q1_k);
           2*(q1_k*q3_k - q0_k*q2_k), 2*(q2_k*q3_k + q0_k*q1_k), q0_k^2 - q1_k^2 - q2_k^2 + q3_k^2];
    
    % Angulos de Euler
    pitch_med(k) = atan2( 2*(q0_k*q1_k + q2_k*q3_k), 1 - 2*(q1_k^2 + q2_k^2));
    roll_med(k)  = asin(2*(q0_k*q2_k - q3_k*q1_k));
    yaw_med(k)   = atan2( 2*(q0_k*q3_k + q1_k*q2_k), 1 - 2*(q2_k^2 + q3_k^2));

    % Rotacion de coordenadas de aceleracion
    acc_b = [a_bx(k) a_by(k) a_bz(k)]';
    acc_W = wR_b * acc_b;
    
    % Vectores de aceleraciones calibradas Frame WORLD
    acc_Wx(k) = acc_W(1)*g_w;
    acc_Wy(k) = acc_W(2)*g_w;
    acc_Wz(k) = acc_W(3)*g_w;  

    aw_k = ([acc_Wx(k); acc_Wy(k); acc_Wz(k)]' + [ 0 0 g_w])'; % Elimina componete de gravedad

    a_norm(k) = sqrt(acc_Wx(k)^2 + acc_Wy(k)^2 + acc_Wz(k)^2); 
    w_norm(k) = sqrt(w_x(k)^2 + w_y(k)^2 + w_z(k)^2); 

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

    Cs(k) = C_1(k) * C_2(k) * C_3(k); %
    %% Fin de condiciones
    
    %% INICIO MULTI-RATE EXTENDED FILTER KALMAN (MR-EFK)
    % comprueba si hay una medicion de gps disponible
    if (gps_disp(k) ~= 0)
        k_gps = k_gps + 1; % gps ON
    end

    if (gps_disp(k) ~= 0 && mod(k_gps, 60) == 0)
        gps_std = 1; % gps ON

        cont_gps = cont_gps + 1;
        tiempo_correcion_gps(cont_gps) = k_cont(k-1);

    else
        gps_std = 0; % gps OFF
    end
    
    v_std = [ ones(2,1)*gps_std ; ones(6,1)*imu_std ]; % Vector de disponibilidad de mediciones
    delta_k = diag(v_std); % Matriz de disponibilidad de mediciones

    A_k = [I I*Ts O zeros(3,4); O I*Cs(k) O zeros(3,4); zeros(7,13)]; % Matriz de Transición parcial del MR-EKF
    B_k = [Ts^2/2 * I O ; Ts * I O]; % Matriz de control

    S = [-q1_k -q2_k -q3_k ; q0_k -q3_k q2_k ; q3_k q0_k -q1_k ; -q2_k q1_k q0_k];

    C_k = [zeros(6,13) ; zeros(4,6) eye(4,4) -Ts/2*S ; zeros(3,6) zeros(3,4) eye(3,3)];
    D_k = [zeros(4,3) Ts/1 * S ; zeros(3,6)];

    PHI = (A_k + C_k); % Matriz de Transición del MR-EKF

    % Prediccion EKF
    x_pred = PHI * x_h + ([B_k ; D_k]) * [aw_k ; W(k,:)']; % x_h = [pos(1:3) , vel(1:3) , QUAT(1:4) , bias_g(1:3)], dim(13,1)
    
    C_a = [-q2_k q3_k -q0_k q1_k ; q1_k q0_k q3_k q2_k ; q0_k -q1_k -q2_k q3_k];
    C_m = [q3_k q2_k q1_k q0_k ; q0_k -q1_k  q2_k -q3_k ; -q1_k -q0_k q3_k q2_k];

    P_pred = PHI * P_h * PHI' + Q_k;

    H = delta_k * [eye(2) zeros(2,11) ; zeros(3,6) C_a O ; zeros(3,6) C_m O]; % dim(8x13)
    z_pred = H * x_pred; % dim(8x1)

    % Actualizacion MR-EKF
    % Medición de acelerometro y magnetometro respectivamente (body-frame)
    acc_b = [a_bx(k) a_by(k) a_bz(k)]; % Acelerometro  
    mag_b = [mag_bx(k) mag_by(k) mag_bz(k)]; % Magnetometro

    m_k = [gps_med_m(k,:) acc_b mag_b]'; % Vector de mediciones {x_gps , y_gps, acc(x,y,z), mag(x,y,z)}
    K_k = P_pred * H' * inv(H * P_pred * H' + R_k);
    x_h = x_pred + K_k * (m_k - z_pred); % Estimación de estados MR-EKF
    P_h = ( eye(13,13) - K_k * H ) * P_pred;
    %% fin del MR-EKF

    k_cont(k) = k*0.1;
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

% polos = eig(A - LC)
cont_gps
N / (cont_gps * 10)
%% FIGURAS
onFig = 1;
if onFig == 1
    figure(1)
    plot(s1,s2, "b")
    hold on
    plot(coord_XY(:,1), coord_XY(:,2), "black") 
    title("Posición MR-EKF")
    legend("MPU9250" , "GPS")
    hold off
    
    figure(2)
    subplot(4,1,1), plot(q0_v), ylim([-1 1]);
    subplot(4,1,2), plot(q1_v), ylim([-1 1]);
    subplot(4,1,3), plot(q2_v), ylim([-1 1]);
    subplot(4,1,4), plot(q3_v), ylim([-1 1]);
    title("Cuaterniones MR-EKF")
    
    figure(3)
    subplot(4,1,1), plot(qc_bno(:,1)), ylim([-1 1]);
    subplot(4,1,2), plot(qc_bno(:,2)), ylim([-1 1]);
    subplot(4,1,3), plot(qc_bno(:,3)), ylim([-1 1]);
    subplot(4,1,4), plot(qc_bno(:,4)), ylim([-1 1]);
    title("Cuaterniones BNO055")
end