% Este programa realiza las siguientes tareas: (1) tomas las coordenadas (x,y) en metro y calcula
% la distancia recorrida y el desplazamiento
function [dist_true, desp_true] = rec_desp(med_m)  
    %% recorrido TRUE
    x_true = med_m(:,2);
    y_true = med_m(:,1);
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
