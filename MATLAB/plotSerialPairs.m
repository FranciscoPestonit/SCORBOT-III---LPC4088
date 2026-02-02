function [tiempo,posiciones] = plotSerialPairs(serialport_data1, doPlot)
% plotSerialPairs  Procesa datos [pos1,dt1,pos2,dt2,...] y opcionalmente grafica
% 
% USO
%   [tiempo,posiciones] = plotSerialPairs(serialport_data1)
%   plotSerialPairs(serialport_data1, true)   % devuelve y además grafica
%
% INPUTS
%   serialport_data1 : vector fila/col con pares [pos, dt]
%   doPlot (opcional) : true/false (por defecto true si no se solicita salida)
%
% OUTPUTS
%   tiempo     : vector de tiempos acumulados (cumsum de dt)
%   posiciones : vector de posiciones (elementos impares)

if nargin < 2
    % si no piden salidas, mostramos la figura por defecto
    doPlot = nargout == 0;
end

% Validaciones y conversión
if isempty(serialport_data1)
    error('Entrada vacía.');
end
data = double(serialport_data1(:)); % columna de dobles

% Eliminar NaN al final si hay un par incompleto marcado por NaN
data = data(~isnan(data));

% Si longitud impar, descartar último elemento incompleto
if mod(numel(data),2) ~= 0
    data(end) = [];
end

% Extraer posiciones e instantes
posiciones = data(1:2:end);
instantes  = data(2:2:end);

% Reemplazar NaN en instantes por 0 (o manejar según convenga)
instantes(isnan(instantes)) = 0;

% Tiempo acumulado
tiempo = cumsum(instantes);

% Asegurar misma longitud (por si algún paso anterior alteró tamaños)
N = min(numel(tiempo), numel(posiciones));
tiempo = tiempo(1:N);
posiciones = posiciones(1:N);

% Plot (si se solicita)
if doPlot
    figure;
    plot(tiempo, posiciones);
    grid on;
    xlabel('Tiempo (ms)');
    ylabel('Pulsos encoder');
    drawnow;
end

end
