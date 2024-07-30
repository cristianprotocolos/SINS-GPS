%clear, clc, close all
%% Datos medidos
global am;
% am = readmatrix("C:\Users\cgon0\Desktop\paper odometria\paraCalibrarACC.csv");
% am = am(:,14:16);


am = rawData;
% Crear un nuevo vector de datos con solo los datos seleccionados
%am = am(indices_seleccionados, :);

biasx = B0mx;
biasy = B0my;
biasz = B0mz;

f_scaling = 0.01;

Jota = fminsearch(@calibracionACCE, [0 0 0 0 0 0 f_scaling f_scaling f_scaling biasx biasy biasz]);
th = Jota;
S_a = [1 th(1) th(2); th(3) 1 th(4); th(5) th(6) 1];
K_a = [th(7) 0 0;0 th(8) 0;0 0 th(9)];
b_a = [th(10) th(11) th(12)]';

for i=1:length(am)
    a(:,i) = S_a*K_a*(am(i,:)' - b_a);
    a_norm(i) = norm(a(:,i));
end

%% PLOT
figure(1)
subplot(4,1,1);plot(a(1, :))
subplot(4,1,2);plot(a(2, :))
subplot(4,1,3);plot(a(3, :))
subplot(4,1,4);plot(a_norm)


%% FUNCIONES
function J = calibracionACCE(th)
    global am;
    em = [1 th(1) th(2); th(3) 1 th(4); th(5) th(6) 1];
    km = [th(7) 0 0;0 th(8) 0;0 0 th(9)];
    b0 = [th(10) th(11) th(12)]';
    J = 0;
        for i =1:length(am)
            a = em*km*(am(i,:)' - b0);
            a_norm(i) = norm(a);
            J = J + (norm(a)-1)^2;    
        end    
%         w1= 1;
%         w2= 1;
%         a_mean = mean(a_norm)
%         a_std = std(a_norm)
%         
%         J = w1*abs(1 - mean(a_norm)) + w2*a_std
end






