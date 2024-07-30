%% Read data calibracion acelerometro
close all, clc, clear
dir = pwd;
% data = readmatrix([dir, '\datos\datosCalibracionACC.csv']); % 14-16
data = readmatrix([dir, '\datos\caminataBlanco_10min_calibracionAcelerometro.csv']); % 14-16
a_b = data(:, 14:16);
m_b = data(:, 20:22);

% figure
% subplot(3,1,1), plot(a_b(:,1))
% subplot(3,1,2), plot(a_b(:,2))
% subplot(3,1,3), plot(a_b(:,3))
% title('Aceleraciones Body-Frame')

for i=1:length(m_b)
    m_norm(i) = norm(m_b(i,:));
end

% for i=1:length(a_b)
%     a_norm(i) = norm(a_b(i,:));
% end
%% datos seleccionados
s1 = (87:128);
s2 = (149:193);
s3 = (216:248);
s4 = (269:296);

s5 = (319:355);
s6 = (381:442);
s7 = (466:535);
s8 = (562:634);

s9 = (686:757);
s10 = (782:868);
s11 = (896:951);
s12 = (990:1093);

s13 = (1148:1216);
s14 = (1295:1365);
s15 = (1408:1463);
s16 = (1519:1590);

s17 = (1627:1663);
s18 = (1751:1836);
s19 = (1882:1992);
s20 = (2032:2070);

s21 = (2109:2193);
s22 = (2237:2397);

for i=1:22
    name = strcat('s', num2str(i));
    selec_data(i , :) = [mean(m_b(eval(name), 1)) , mean(m_b(eval(name), 2)) , mean(m_b(eval(name), 3))]
end
figure
subplot(4,1,1), plot(m_b(:,1))
subplot(4,1,2), plot(m_b(:,2))
subplot(4,1,3), plot(m_b(:,3))
subplot(4,1,4), plot(m_norm)
title('Magnetometro Body-Frame')

