function coordenadas_metros = convertir_GPS_a_metros(lista_gps, coordenada_referencia)
    % lista_gps: Lista de coordenadas GPS a convertir, cada fila representa una coordenada [latitud, longitud]
    % coordenada_referencia: Coordenada GPS de referencia [latitud_ref, longitud_ref]

    % Radio de la Tierra en metros
    R = 6371000;

    % Convertir la coordenada de referencia a radianes
    lat_ref = deg2rad(coordenada_referencia(1));
    lon_ref = deg2rad(coordenada_referencia(2));

    % Calcular las diferencias de latitud y longitud respecto a la coordenada de referencia
    delta_lat = deg2rad(lista_gps(:,1)) - lat_ref;
    delta_lon = deg2rad(lista_gps(:,2)) - lon_ref;

    % Calcular las distancias en metros respecto a la coordenada de referencia
    distancia_norte = R * delta_lat;
    distancia_este = R * cos(lat_ref) .* delta_lon;

    % Coordenadas en metros respecto a la coordenada de referencia
    coordenadas_metros = [distancia_norte, distancia_este];
end
