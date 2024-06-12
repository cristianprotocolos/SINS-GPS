% constantes.m

% parametros de calibracion del acelerometro 
th_acc = [0.00348071285001877, -0.000785715436595477, -0.00304916604691251, ...
         -0.00166593786338318, 0.00132809919438384, 0.00167220420183682, ... 
         -6.09943696144438e-05, -6.06099273282422e-05, -5.97951315870594e-05, ... 
          555.932711506898, 147.042065618102, 174.935127393677]; %% MPU9250

% th_acc = [0.00348071285001877, -0.000785715436595477, -0.00304916604691251, ...
%          -0.00166593786338318, 0.00132809919438384, 0.00167220420183682, ... 
%          0.00103606586064220, 0.0010356721689052, 0.00103210836111909, ... 
%           8.31847270472375, -21.9462961335939, 20.5860087074609]; %% BNO055

S_a = [ 1 th_acc(1) th_acc(2); th_acc(3) 1 th_acc(4); th_acc(5)	th_acc(6) 1];
k_a = diag([th_acc(7) th_acc(8) th_acc(9)]);
bias_a = [th_acc(10) th_acc(11) th_acc(12)]';

%% constantes
g_w = 1; % magnitud aceleración de gravedad
Ts = 0.1; % tasa de muestreo
N = length(am);
T = dt*N;
dt_gps = 5;
correcOn = 1;

%% Creacion vectores de velocidad
Vx_k = zeros(1,N);
Vy_k = zeros(1,N);
Vz_k = zeros(1,N);

Vx_body = zeros(1,N);
Vy_body = zeros(1,N);
Vz_body = zeros(1,N);

%% Creacion vectores de posición
Sx_k = zeros(1,N);
Sy_k = zeros(1,N);
Sz_k = zeros(1,N);

Sx_body = zeros(1,N);
Sy_body = zeros(1,N);
Sz_body = zeros(1,N);

%% Matrices y vectores del filtro de Kalman extendido
x_h = [0 0 0 0 0]'; % x_h = [s1 s2 v1 v2 yaw] inicializacion  
I = eye(2);
O = zeros(2,2);
Cs = c; % Condicion de parada
eul = 0;
aw_k = [1 1]';


