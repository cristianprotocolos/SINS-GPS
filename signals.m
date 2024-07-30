%% señales
close all, clear, clc

dataIMU
constantes

dir = pwd;
data = readmatrix([dir, '\datos\caminataBlanco_13min.csv']);
N_data = length(data);
%% BNO055: ace(1:3) , mag(4:6) , giro(7:9) , quat(10:13)
data_bno = data(:, 1:13);
ab_bno = data_bno(:, 1:3)';
mb_bno = data_bno(:, 4:6);
g_bno = data_bno(:, 7:9);
quat_bno = data_bno(:, 10:13);

%% MPU9250: ace(14:16) , giro(17:19) , mag(20:22)
data_mpu = data(:, 14:22);
ab_mpu = data_mpu(:, 1:3);
g_mpu = data_mpu(:, 4:6);
mb_mpu = data_mpu(:, 7:9);

%% Calibracion de Aceleraciones en body-frame MPU9250
for i=1:N_data
    a_b(:,i) = S_a * k_a * (ab_mpu(i,:)' - bias_a) * g_w; % Acelerometro Body-Frame
    ab_norm(i) = norm(a_b(:,i));

    m_b(:,i) = S_mag * k_mag * (mb_mpu(i,:)' - bias_mag); % Magnetometro Body-Frame
    mb_norm(i) = norm(m_b(:,i));
end

ab_sim = [ones(1,N_data)*5 ; ones(1,N_data)*5 ; ones(1,N_data)*9.8 ];

q0_s = [ones(1,N_data)*0.7071];
q1_s = [ones(1,N_data)*0.7071];
q2_s = [ones(1,N_data)*0];
q3_s = [ones(1,N_data)*0];


%% Rotacion frame de Aceleraciones a world-frame MPU9250
for k=1:N_data

    q_scaling = norm([quat_bno(k, 1) quat_bno(k, 2) quat_bno(k, 3) quat_bno(k, 4)]);

    q0_k = quat_bno(k, 4) / q_scaling;  
    q1_k = quat_bno(k, 3) / q_scaling;  
    q2_k = quat_bno(k, 2) / q_scaling;  
    q3_k = quat_bno(k, 1) / q_scaling;  

    wR_b = [q0_k^2 + q1_k^2 - q2_k^2 - q3_k^2, 2*(q1_k*q2_k - q0_k*q3_k), 2*(q1_k*q3_k + q0_k*q2_k);
            2*(q1_k*q2_k + q0_k*q3_k), q0_k^2 - q1_k^2 + q2_k^2 - q3_k^2, 2*(q2_k*q3_k - q0_k*q1_k);
            2*(q1_k*q3_k - q0_k*q2_k), 2*(q2_k*q3_k + q0_k*q1_k), q0_k^2 - q1_k^2 - q2_k^2 + q3_k^2];
    
    a_w(:,k) =  wR_b * a_b(:,k);
    aw_norm(k) = norm(a_w(:,k));

    aw_bno(:,k) =  wR_b * [0 norm( ab_bno(1,k) , ab_bno(2,k) ) ab_bno(3,k)]';
    

    q0_sim = q0_s(k);
    q1_sim = q1_s(k);
    q2_sim = q2_s(k);
    q3_sim = q3_s(k);

    R_sim = [q0_sim^2 + q1_sim^2 - q2_sim^2 - q3_sim^2, 2*(q1_sim*q2_sim - q0_sim*q3_sim), 2*(q1_sim*q3_sim + q0_sim*q2_sim);
            2*(q1_sim*q2_sim + q0_sim*q3_sim), q0_sim^2 - q1_sim^2 + q2_sim^2 - q3_sim^2, 2*(q2_sim*q3_sim - q0_sim*q1_sim);
            2*(q1_sim*q3_sim - q0_sim*q2_sim), 2*(q2_sim*q3_sim + q0_sim*q1_sim), q0_sim^2 - q1_sim^2 - q2_sim^2 + q3_sim^2];
    
    asim_w(:,k) =  R_sim * ab_sim(:,k);

end

%% Adaptative extended filter kalman

%% Modelo de predicion de orientacion
    % q0 = x_h(1,k);
    % q1 = x_h(2,k);
    % q2 = x_h(3,k);
    % q3 = x_h(4,k);
    % 
    % bg_x = x_h(5,k);
    % bg_y = x_h(6,k);
    % bg_z = x_h(7,k);
    % 
    % bias_g = [bg_x bg_y bg_z]';
    % 
    % % Coreccion de Bias giroscopio
    % wb(:, k) = Scaling_g * ( wm(k,:)' - bias_g );
    % wb_norm(k) = norm(wb(:, k));
    % 
    % Q_q = [-q1 -q2 -q3 ; q0 -q3 q2 ; q3 q0 -q1 ; -q2 q1 q0];
    % A_q = [eye(4) -Ts/2*Q_q ; zeros(3,4) eye(3)];
    % B_q = [Ts/2 * Q_q ; zeros(3)];
    % 
    % C_a = [-q2 q3 -q0 q1 ; q1 q0 q3 q2 ; q0 -q1 -q2 q3];
    % C_m = [q3 q2 q1 q0 ; q0 -q1 q2 -q3 ; -q1 -q0 q3 q2];
    % C_q = [C_a zeros(3,3) ; C_m zeros(3,3)];
    % 
    % u_q = wb(:, k);
    % 
    % x_pred = A_q * x_h(:, k) + B_q * u_q; 
    % P_pred = A_q * P_h * A_q' + Q_qa;   
    % y_pred = C_q * x_pred;
    % 
    % % Correccion AD-EKF
    % y_med = [a_b(:,k) mag_b(:,k)']'; % Vector de mediciones {acc(x,y,z), mag(x,y,z)}
    % 
    % K_k = P_pred * C_q' * inv(C_q * P_pred * C_q' + R_q);
    % 
    % x_h(:, k+1) = x_pred + K_k * (y_med - y_pred); % Estimación de estados MR-EKF
    % 
    % P_h = (eye(7) - K_k * C_q ) * P_pred;


% obtener velocidad 2044 a 2060

v = zeros(3,32);
s = zeros(3,32);

ini = 2337;
fin = 2354;

a_w_mean = [mean(a_w(1, ini:fin )) mean(a_w(2, ini:fin )) mean(a_w(3, ini:fin ))]';

for i=ini:fin
    v(:,i - ini + 2) = v(:,i - ini + 1) + Ts * (a_w(:,i) - a_w_mean);
end

for i=ini:fin
    s(:,i - ini + 2) = s(:,i - ini + 1) + Ts* v(:,i - ini +1);
end

for i=1:length(s)
    s_norm(i) = norm([s(1,i) s(2,i)]);
    v_norm(i) = norm([v(1,i) v(2,i)]);
end
figure
subplot(4,1,1), plot(v(1,:))
subplot(4,1,2), plot(v(2,:))
subplot(4,1,3), plot(v(3,:))
subplot(4,1,4), plot(v_norm)
title('Velocidades')

figure
subplot(4,1,1), plot(s(1,:))
subplot(4,1,2), plot(s(2,:))
subplot(4,1,3), plot(s(3,:))
subplot(4,1,4), plot(s_norm)
title('Posicion')
% rectas horizontales

y = ones(7200,1); 

%% figuras BNO055
mpu_fig = 1;
bno_fig = 0;
x_min = ini-10;
x_max = fin+10;
if bno_fig == 1
    figure
    subplot(3,1,1), plot(ab_bno(1,:))
    subplot(3,1,2), plot(ab_bno(2,:))
    subplot(3,1,3), plot(ab_bno(3,:))
    title('Aceleraciones Body-Frame')
    
    figure
    subplot(3,1,1), plot(aw_bno(1,:)*0.001)
    subplot(3,1,2), plot(aw_bno(2,:)*0.001)
    subplot(3,1,3), plot(aw_bno(3,:)*0.001)
    title('Aceleraciones World-Frame')

    figure
    subplot(3,1,1), plot(asim_w(1,:))
    subplot(3,1,2), plot(asim_w(2,:))
    subplot(3,1,3), plot(asim_w(3,:))
    title('Aceleraciones sim World-Frame')
    
    % figure
    % subplot(3,1,1), plot(mb_bno(:,1))
    % subplot(3,1,2), plot(mb_bno(:,2))
    % subplot(3,1,3), plot(mb_bno(:,3))
    % title('Fuerza magnetica Body-Frame')
    
    % figure
    % subplot(3,1,1), plot(g_bno(:,1))
    % subplot(3,1,2), plot(g_bno(:,2))
    % subplot(3,1,3), plot(g_bno(:,3))
    % title('Velocidad angular')
    
    figure
    subplot(4,1,1), plot(quat_bno(:,1))
    subplot(4,1,2), plot(quat_bno(:,2))
    subplot(4,1,3), plot(quat_bno(:,3))
    subplot(4,1,4), plot(quat_bno(:,4))
    title('Cuaterniones BNO055')
end

if mpu_fig == 1
    %% figuras MPU 9250
    x_min = 1;
    x_max = N_data;
    figure
    subplot(4,1,1), plot(a_b(1,:)), xlim([x_min x_max])
    subplot(4,1,2), plot(a_b(2,:)), xlim([x_min x_max])
    subplot(4,1,3), plot(a_b(3,:)), xlim([x_min x_max])
    subplot(4,1,4), plot(ab_norm) , xlim([x_min x_max])    
    title('Aceleraciones Body-Frame')
    
    figure
    subplot(4,1,1), plot(a_w(1,:) - mean(a_w(1, ini:fin ))), xlim([x_min x_max]), hold on , plot(y * 0, '-k', 'LineWidth', 1)
    subplot(4,1,2), plot(a_w(2,:) - mean(a_w(2, ini:fin ))), xlim([x_min x_max]), hold on , plot(y * 0, '-k', 'LineWidth', 1)
    subplot(4,1,3), plot(a_w(3,:)), xlim([x_min x_max]), hold on , plot(y * g_w, '-k', 'LineWidth', 1)
    subplot(4,1,4), plot(aw_norm) , xlim([x_min x_max]), hold on , plot(y * g_w, '-k', 'LineWidth', 1)
    title('Aceleraciones World-Frame')

    % figure
    % subplot(4,1,1), plot(a_w(1,:))
    % subplot(4,1,2), plot(a_w(2,:))
    % subplot(4,1,3), plot(a_w(3,:))
    % subplot(4,1,4), plot(aw_norm)
    % title('Aceleraciones World-Frame')
%     figure
%     subplot(3,1,1), plot(g_mpu(:,1)), xlim([x_min x_max])
%     subplot(3,1,2), plot(g_mpu(:,2)), xlim([x_min x_max])
%     subplot(3,1,3), plot(g_mpu(:,3)), xlim([x_min x_max])
%     title('Velocidad angular')
    
    % figure
    % subplot(4,1,1), plot(m_b(1,:))
    % subplot(4,1,2), plot(m_b(2,:))
    % subplot(4,1,3), plot(m_b(3,:))
    % subplot(4,1,4), plot(mb_norm)
    % title('Fuerza magnetica Body-Frame')

    % figure
    % subplot(3,1,1), plot(mb_mpu(:,1))
    % subplot(3,1,2), plot(mb_mpu(:,2))
    % subplot(3,1,3), plot(mb_mpu(:,3))
    
    figure
    subplot(4,1,1), plot(quat_bno(:, 1) / q_scaling), ylim([-1 1])
    subplot(4,1,2), plot(quat_bno(:, 2) / q_scaling), ylim([-1 1])
    subplot(4,1,3), plot(quat_bno(:, 3) / q_scaling), ylim([-1 1])
    subplot(4,1,4), plot(quat_bno(:, 4) / q_scaling), ylim([-1 1])
    title('Quaterniones BNO055')
end