%function [B0mx, B0my, B0mz] = codDMA
%clear, clc, close all
filedate = '20160512';
filetime = '145652';
filedir = '.';
filename = '20160630-magnetometer-raw-data-II.csv';
headerLines = 1;

X = []; 
Y = []; 
XtX = []; 
XtY = []; 

a11 = 0.;
a12 = 0.;
a13 = 0.;
a14 = 0.;
a21 = 0.;
a22 = 0.;
a23 = 0.;
a24 = 0.;
a31 = 0.;
a32 = 0.;
a33 = 0.;
a34 = 0.;
a41 = 0.;
a42 = 0.;
a43 = 0.;
a44 = 0.;
b11 = 0.;
b21 = 0.;
b31 = 0.;
b41 = 0.;

dir = pwd;
data = readmatrix([dir, '\datos\calibracionMagnetometro.csv']);
N_data = length(data);
%% 20 a 22 corresponde a magnetometro de la mpu9250
%% 14 a 16 corresponde a acelerometro de la mpu9250
rawData = data(:, 20:22);

Bx = rawData(:,1); 
By = rawData(:,2); 
Bz = rawData(:,3); 


numberReadings = size(Bx); 
for i=1:numberReadings(1)

a11 = a11 + Bx(i)*Bx(i);
a12 = a12 + Bx(i)*By(i);
a13 = a13 + Bx(i)*Bz(i);
a14 = a14 + Bx(i);

a21 = a21 + By(i)*Bx(i);
a22 = a22 + By(i)*By(i);
a23 = a23 + By(i)*Bz(i);
a24 = a24 + By(i);

a31 = a31 + Bz(i)*Bx(i);
a32 = a32 + Bz(i)*By(i);
a33 = a33 + Bz(i)*Bz(i);
a34 = a34 + Bz(i);

a41 = a41 + Bx(i);
a42 = a42 + By(i);
a43 = a43 + Bz(i);
a44 = a44 + 1;

b11 = b11 + Bx(i)*(Bx(i)*Bx(i)+By(i)*By(i)+Bz(i)*Bz(i));
b21 = b21 + By(i)*(Bx(i)*Bx(i)+By(i)*By(i)+Bz(i)*Bz(i));
b31 = b31 + Bz(i)*(Bx(i)*Bx(i)+By(i)*By(i)+Bz(i)*Bz(i));
b41 = b41 + (Bx(i)*Bx(i)+By(i)*By(i)+Bz(i)*Bz(i));
end

XtX = [a11 a12 a13 a14; a21 a22 a23 a24; a31 a32 a33 a34; a41 a42 a43 a44];
XtY = [b11; b21; b31; b41];

beta = inv(XtX) * XtY;

B0mx = 1/2 * beta(1);
B0my = 1/2 * beta(2);
B0mz = 1/2 * beta(3);

b_m = [B0mx B0my B0mz]';
for i=1:length(rawData)
    a = [0.0275552545910399 0 0; 0 0.027737 0; 0 0 0.027466]*( rawData(i,:)' - b_m );
    ax(i) = a(1);
    ay(i) = a(2);
    az(i) = a(3);
    magACE(i) = norm(a);
end

rangoX = max(Bx-B0mx) - min(Bx-B0mx)
rangoY = max(By-B0my) - min(By-B0my)
rangoZ = max(Bz-B0mz) - min(Bz-B0mz)

fc = 1;

figure(1)
subplot(4,1,1), plot((Bx-B0mx)*fc)
subplot(4,1,2), plot((By-B0my)*fc)
subplot(4,1,3), plot((Bz-B0mz)*fc)
subplot(4,1,4), plot(magACE)

figure(2)
subplot(3,1,1), plot(Bx)
subplot(3,1,2), plot(By)
subplot(3,1,3), plot(Bz)

figure(3)
histogram(magACE)



%%%%%%%%%%%%%%%%%%%%

% Calcular la media y la desviación estándar de los datos
datos = magACE;
media = mean(datos);
desviacion_estandar = std(datos);

% Definir los límites del rango dentro de ±2 desviaciones estándar
limite_inferior = media - 1 * desviacion_estandar;
limite_superior = media + 1 * desviacion_estandar;

% Seleccionar los datos dentro del rango
datos_seleccionados = datos(datos >= limite_inferior & datos <= limite_superior);

% Seleccionar los índices de los datos dentro del rango
indices_seleccionados = find(datos >= limite_inferior & datos <= limite_superior);

% Crear un nuevo vector de datos con solo los datos seleccionados
nuevos_datos = datos(indices_seleccionados);

% Graficar histograma de los datos originales y los datos seleccionados
figure;
subplot(2, 1, 1);
histogram(datos, 'Normalization', 'probability');
title('Histograma de Todos los Datos');
xlabel('Valor');
ylabel('Frecuencia Relativa');

subplot(2, 1, 2);
histogram(datos_seleccionados, 'Normalization', 'probability');
title('Histograma de Datos Seleccionados');
xlabel('Valor');
ylabel('Frecuencia Relativa');


