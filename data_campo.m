% dataIMU.m
clear 
clc
close all

dir = pwd;
gps_campo = readtable([dir, '\datos\datos_campo\20240601212648'])

lecheria = [-38.836556207956754, -72.69535912332917];


inicio = 1:800;

lat_campo = table2array(gps_campo(:,6));
lat_campo = str2double(strrep(lat_campo, ',', ''));

lon_campo = table2array(gps_campo(:,9));
lon_campo = str2double(strrep(lon_campo, ',', ''));

figure
plot(lat_campo)

figure
plot(lon_campo)

figure
plot(lat_campo , lon_campo)
hold on
plot(lecheria(1) , lecheria(2), 'ro', 'MarkerFaceColor', 'r')

