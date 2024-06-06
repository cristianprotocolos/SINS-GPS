%% Esta funcion realiza estas tareas en serie: convierte coordenadas GPS en distancia en unidades de
%% metro(1), luego calcula la distancia recorrida(2) y el desplazamiento(3).
function [dist_true, desp_true, coord_XY] = gps_med2m(gps_true) 
    coordenadas_metros_true = convertir_GPS_a_metros(gps_true, gps_true(1,:));    
    %% recorrido TRUE
    x_true = coordenadas_metros_true(:,2);
    y_true = coordenadas_metros_true(:,1);
    coord_XY = [x_true y_true];
    N = length(x_true);
    distancia_total = 0;    
    for k = 1:N-1
        % Calcular la distancia euclidiana entre puntos consecutivos  
        distancia_total = distancia_total + norm([x_true(k+1) - x_true(k), y_true(k+1) - y_true(k)]);
        dist_true(k) =  distancia_total;
    end    
    %% desplazamiento TRUE
    ref_true = [x_true(1), y_true(1)];
    N_true = length(x_true);
    for k = 1:N_true-1
        des_k = [x_true(k), y_true(k)] - ref_true;
        desp_true(k) =  norm(des_k);
    end
end