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

%% EKF
for k=1:N
    %% Modelo de predicion de orientacion
    q0 = x_h(1);
    q1 = x_h(2);
    q2 = x_h(3);
    q3 = x_h(4);
    
    q = [q0 q1 q2 q3];
    eulerAngles = quaternionToEulerAngles(q);
    yaw(k) = eulerAngles(3);

    q0_v(k) = x_h(1);
    q1_v(k) = x_h(2);
    q2_v(k) = x_h(3);
    q3_v(k) = x_h(4);

    bg_x = x_h(5);
    bg_y = x_h(6);
    bg_z = x_h(7);

    % Coreccion de Bias giroscopio
    w_x = (w_body(k,1) + bg_x)* Scaling_g;
    w_y = (w_body(k,2) + bg_y)* Scaling_g;
    w_z = (w_body(k,3) + bg_z)* Scaling_g;

    x_q = [x_h(1) x_h(2) x_h(3) x_h(4) x_h(5) x_h(6) x_h(7)]';
    Q_q = [-q1 -q2 -q3 ; q0 -q3 q2 ; q3 q0 -q1 ; -q2 q1 q0];
    A_q = [eye(4) -Ts/2*Q_q ; zeros(3,4) eye(3)];
    B_q = [Ts/2 * Q_q ; zeros(3)];
    
    C_a = [-q2 q3 -q0 q1 ; q1 q0 q3 q2 ; q0 -q1 -q2 q3];
    C_m = [q3 q2 q1 q0 ; q0 -q1 q2 -q3 ; -q1 -q0 q3 q2];
    C_q = [C_a zeros(3,3) ; C_m zeros(3,3)];

    u_q = [w_x w_y w_z]';

    %% Modelo de velocidad y posicion

    %% CONDICIONES
    a_norm(k) = sqrt(a_bx(k)^2 + a_bx(k)^2 + a_bx(k)^2); 
    w_norm(k) = sqrt(w_x^2 + w_y^2 + w_z^2);
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
    
    % Matriz de rotacion b --> W 
    R11 = q0^2 + q1^2 - q2^2 - q3^2;
    R12 = 2*(q1*q2 - q0*q3);
    R13 = 2*(q1*q3 + q0*q2);
    R21 = 2*(q1*q2 + q0*q3);
    R22 = q0^2 - q1^2 + q2^2 - q3^2;
    R23 = 2*(q2*q3 - q0*q1);
    
    R31 = 2*(q1*q3 + q0*q1);
    R32 = 2*(q2*q3 - q0*q1);
    R33 = q0^2 - q1^2 - q2^2 + q3^2;

    wR_b = [R11 , R12 , R13;
            R21, R22 , R23];

    %
    u_p = [a_bx(k) a_by(k) a_bz(k)]';

    aw_x(k) = [R11 , R12 , R13] * u_p;
    aw_y(k) = [R21 , R22 , R23] * u_p;
    aw_z(k) = [R31 , R32 , R33] * u_p;

    A_p = [I I*Ts I O ; O I*Cs(k) O I ; O O I O ; O O O I];
    B_p = [Ts^2/2 * wR_b ; Ts * wR_b ; zeros(2,3); zeros(2,3)];
    C_p = [I zeros(2,6)];

    
    % figure(12), 
    % subplot(3,1,1),plot(aw_x), ylim([-0.5 0.5]), xlim([k-50 k])
    % 
    % subplot(3,1,2),plot(aw_y), ylim([-0.5 0.5]), xlim([k-50 k])
    % 
    % subplot(3,1,3),plot(aw_z), ylim([0.3 1.3]), xlim([k-50 k])
   
   
    %% INICIO MULTI-RATE EXTENDED FILTER KALMAN (MR-EFK)
    % comprueba si hay una medicion de gps disponible
    if (gps_disp(k) ~= 0)
        k_gps = k_gps + 1; % gps ON
    end

    if (gps_disp(k) ~= 0 && mod(k_gps, periodoGPS) == 0)
        gps_std = 1; % gps ON

        cont_gps = cont_gps + 1;
        tiempo_correcion_gps(cont_gps) = k_cont(k-1);

    else
        gps_std = 0; % gps OFF
    end
    
    v_std = [ ones(2,1)*gps_std]; % Vector de disponibilidad de mediciones
    delta_k = diag(v_std); % Matriz de disponibilidad de mediciones


    A_k = [A_q zeros(7,8) ; zeros(8,7) A_p]; % Matriz de Transición parcial del MR-EKF
    B_k = [B_q zeros(7,3) ; zeros(8,3) B_p]; % Matriz de control

    C_k = [C_q zeros(6,8) ; zeros(2,7) delta_k*C_p];
    u_k = [u_q ; u_p];

    % Prediccion MR-EKF
    x_pred = A_k * x_h + B_k * u_k; % x_h = [pos(1:3) , vel(1:3) , QUAT(1:4) , bias_g(1:3)], dim(13,1)
    y_pred = C_k * x_pred; % dim(8x1)
    
    P_pred = A_k * P_h * A_k' + Q_k;   

    % Correccion MR-EKF
    % Medición de acelerometro y magnetometro respectivamente (body-frame)
    acc_b = [a_bx(k) a_by(k) a_bz(k)]; % Acelerometro  
    mag_b = [mag_bx(k) mag_by(k) mag_bz(k)]; % Magnetometro

    y_med = [acc_b mag_b delta_k(1) * gps_med_m(k,:)]'; % Vector de mediciones {x_gps , y_gps, acc(x,y,z), mag(x,y,z)}

    K_k = P_pred * C_k' * inv(C_k * P_pred * C_k' + R_k);

    x_h = x_pred + K_k * (y_med - y_pred); % Estimación de estados MR-EKF

    P_h = (eye(15) - K_k * C_k ) * P_pred;
    %% fin del MR-EKF
    k;
    k_cont(k) = k*0.1;

    s1(k) = x_h(8);
    s2(k) = x_h(9);

    v1(k) = x_h(10);
    v2(k) = x_h(11);
end

%% calculo de angulos de mediciones GPS
% for i=3:length(coord_XY_med((1:end),1))
%     ref = [0, 1];  
%     if i==1
%         v1 = [0, 0];
%     else
%         v1 = [coord_XY_med(i-1,1), coord_XY_med(i-1,2)];
%     end
%     v2 = [coord_XY_med(i,1), coord_XY_med(i,2)]; 
%     v = v2 - v1;
%     % Calcular el producto punto entre el vector v y el eje x
%     dot_product = dot(v, ref);
%     % Calcular la magnitud de los vectores
%     magnitude_v = norm(v);
%     magnitude_x_axis = norm(ref);
%     % Calcular el ángulo usando la definición del producto punto
%     cos_theta = dot_product / (magnitude_v * magnitude_x_axis);
%     % Calcular el ángulo en radianes
%     theta_rad = acos(cos_theta);
%     % Convertir el ángulo a grados
%     theta_deg = rad2deg(theta_rad);
%     angleGPS(i) = theta_deg;
%     
% end

% polos = eig(A - LC)
cont_gps
N / (cont_gps * 10)
disp(sprintf("Tiempos de correccion %d" , N / (cont_gps * 10)))

%% FIGURAS
onFig = 1;
if onFig == 1

    figure(1)
    plot(s1(2000:end),s2(2000:end), "b")
    hold on
    plot(coord_XY_med(:,1), coord_XY_med(:,2), "black") 
    %plot(coord_XY_true(:,1), coord_XY_true(:,2), "r") 

    title("Posición MR-EKF")
    legend("MPU9250" , "GPS", "True")
    hold off
    
%     figure(2)
%     subplot(4,1,1), plot(-q3_v), ylim([-1 1]);
%     subplot(4,1,2), plot(q1_v), ylim([-1 1]);
%     subplot(4,1,3), plot(q2_v), ylim([-1 1]);
%     subplot(4,1,4), plot(-q0_v), ylim([-1 1]);
%     title("Cuaterniones MR-EKF")
% %     
%     figure(3)
%     subplot(4,1,1), plot(qc_bno(:,1)), ylim([-1 1]);
%     subplot(4,1,2), plot(qc_bno(:,2)), ylim([-1 1]);
%     subplot(4,1,3), plot(qc_bno(:,3)), ylim([-1 1]);
%     subplot(4,1,4), plot(qc_bno(:,4)), ylim([-1 1]);
%     title("Cuaterniones BNO055")
% 
%     figure(4)
%     subplot(2,1,1), plot(v1), ylim([-2 2]);
%     subplot(2,1,2), plot(v2), ylim([-2 2]);
%     title("Velocidades x y")
% 
%     figure(5), plot(yaw)
% 
%     yawDeg = rad2deg(yaw)
%     % for i=1:length(yawDeg)
%     %     if yawDeg(i) < 0
%     %         yawDeg(i) = yawDeg(i) + 360;
%     %     end
%     % end
%     figure(6), plot(abs(yawDeg))
end

c1_sum = sum(C_1);
c2_sum = sum(C_2);
c3_sum = sum(C_3);
Cs_sum = sum(Cs);

%% ERRORES
T_gps = linspace(1,100, length(coord_XY_med(:,1)));
T_fk = linspace(1,100, length(s1));

% Interpolar la señal 2 para que coincida con la frecuencia de muestreo de la señal 1
gps_x = interp1(T_gps, coord_XY_med(:,1), T_fk);
gps_y = interp1(T_gps, coord_XY_med(:,2), T_fk);

% figure(2), plot(gps_x, gps_y)

ini = 2000;

ERROR_X = sum(sqrt((s1(ini:end) - gps_x(ini:end)).^2))/length(s1)
ERROR_Y = sum(sqrt((s2(ini:end) - gps_y(ini:end)).^2))/length(s2)

ERROR_MAX_X = max(sqrt((s1(ini:end) - gps_x(ini:end)).^2))
ERROR_MAX_Y = max(sqrt((s2(ini:end) - gps_y(ini:end)).^2))

[M,I] = max(sqrt((s1(ini:end) - gps_x(ini:end)).^2))

figure
plot(s1(ini:end)), hold on, plot(gps_x(ini:end)), legend('IMU','GPS'), title('s_1 vs gps_x')

figure
plot(s2(ini:end)), hold on, plot(gps_y(ini:end)), legend('IMU','GPS'), title('s_2 vs gps_y')

function eulerAngles = quaternionToEulerAngles(q)
    % Esta función convierte un cuaternión a ángulos de Euler (radianes).
    % El cuaternión q debe estar en la forma [w, x, y, z].

    % Extraer los componentes del cuaternión
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    % Calcular los ángulos de Euler
    % Roll (phi)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    % Pitch (theta)
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi / 2; % Utilizar 90 grados si fuera de rango
    else
        pitch = asin(sinp);
    end

    % Yaw (psi)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

    % Devolver los ángulos de Euler
    eulerAngles = [roll, pitch, yaw];
end