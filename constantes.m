% constantes.m

%% parametros de calibracion del acelerometro 
% th_acc = [0.000650766958035680	0.00296136438202472	0.000948414825523089 ...
%           0.00384952475550117	-0.00175266416990394	0.00151224446488877 ...
%           6.09400524650312e-05	6.08570376171603e-05	6.05263178941195e-05 ...
%           252.427980648721	313.727047074641	1029.31728793547]; %% MPU9250


th_acc = [0.0142785725115173	-0.000565262782874847	-0.0135137133027348 ...
          0.00113822010433714	-3.50766301955667e-05	0.00529355716043407	... 
          6.09437351909688e-05	6.08947452925976e-05	6.05888936053600e-05 ...
          219.603796971259	310.397193613473	1020.49187309135]; %% MPU9250

% th_acc = [0.00348071285001877, -0.000785715436595477, -0.00304916604691251, ...
%          -0.00166593786338318, 0.00132809919438384, 0.00167220420183682, ... 
%          0.00103606586064220, 0.0010356721689052, 0.00103210836111909, ... 
%           8.31847270472375, -21.9462961335939, 20.5860087074609]; %% BNO055
S_a = [ 1 th_acc(1) th_acc(2); th_acc(3) 1 th_acc(4); th_acc(5)	th_acc(6) 1];
k_a = diag([th_acc(7) th_acc(8) th_acc(9)]);
bias_a = [th_acc(10) th_acc(11) th_acc(12)]';

%% parametros de calibracion del magnetometro 

% th_mag = [0.00756123818798812	0.00750216420284704	0.00757777259204762, ...
%          69.9366643650663	61.4169532692069	88.4271633142602]; %% MPU9250 caja negra


th_mag = [-0.0623382985012097	-0.0523238945200850	0.0627371542820727	...
           0.0578609379131401	0.0440774850602961	-0.00469842883490336	...
           0.00745165183847368	0.00745405157690669	0.00752457907635970	...
           73.0466585465595	70.6978470294840	77.2693070435128]; %% MPU9250


S_mag = [ 1 th_mag(1) th_mag(2); th_mag(3) 1 th_mag(4); th_mag(5)	th_mag(6) 1];
k_mag = diag([th_mag(7) th_mag(8) th_mag(9)]);
bias_mag = [th_mag(10) th_mag(11) th_mag(12)]';

%% constantes
 % magnitud aceleración de gravedad
Ts = 0.1; % tasa de muestreo
dt = Ts;
N = length(am);
T = Ts*N;
dt_gps = 5;
correcOn = 1;
g_w = 9.81;

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

%% Matrices y vectores del multi-rate filtro de Kalman extendido
x_h = zeros(7,7200);
%x_h(:,1) = [1 0 0 0 0 0 0 zeros(1,4) 0.0115 -0.0210 -0.1144 0.1588]';
x_h(:,1) = [1 0 0 0 0 0 0]';

I = eye(2);
O = zeros(2);

eul = 0;
delta_k = eye(8); % tamaño definido por la cantidad de mediciones posibles: x_gps, y_gps, acc(3) y b(3), total 9
gps_std = 1;
imu_std = 1;

P_h = eye(7)*1e3;

Q_qa = ones(1,7)*1e-4; % asociada al giroscopio
Q_p = ones(1,8)*1e-0; % asociada al modelo de prediccion de posicion

R_q = ones(1,6)*1e-3; % asociada al acelerometro y magnetometro
R_p = ones(1,2)*1e-3; % asociada al GPS

Q_k = diag([Q_qa , Q_p]);
R_k = diag([R_q , R_p]);
%% Parametros condiones C1, C2 y C3
Cs = zeros(1, N); % Condicion de parada
aw_k = [0 0 0]';
thrhd_amin =  g_w - 0.05*g_w;
thrhd_amax =  g_w + 0.05*g_w;
% C2
thrhdS = 0.1;
s = 5; % ventana de promedio de tamaño 5
window_size = 2*s + 1;
% C3
thrhdwmax = 0.1;
k_gps = 1;
cont_gps = 0;
periodoGPS = 20;
thvmax = 0.9;
thvmin = -0.9;

%Tiempos
tiempo_gps = linspace(1,100, 3900);
tiempo_imu = linspace(1,100, 20000);

th_x = 0.90000;
th_y = 0.9000;