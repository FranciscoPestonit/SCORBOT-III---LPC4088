function T = cinematicaDirectaSTLV3(q)
    % === Cinematica directa y visualizacion 3D con STL ===
    % q: vector de angulos articulares [q1 q2 q3 q4 q5] en grados
    % Devuelve la matriz homogenea final T (pose de la pinza)

    % --- Paso 1: Pasar q a radianes ---
    q = q * pi/180;

    % --- Paso 2: Parametros DH ---
    teta = [q(1), q(2)+pi/2+pi/6, -q(2)+q(3)-pi/2-pi/6, q(4)-q(3), 0];  
    d    = [35 0 0 0 14];  
    alfa = [pi/2 0 0 pi/2 0];  
    a    = [1.5 22.2 22.2 0 0];

    % --- Paso 3: Matrices homogeneas individuales ---
    A01 = denavit(teta(1), d(1), a(1), alfa(1));
    A12 = denavit(teta(2), d(2), a(2), alfa(2));
    A23 = denavit(teta(3), d(3), a(3), alfa(3));
    A34 = denavit(teta(4), d(4), a(4), alfa(4));
    A45 = denavit(teta(5), d(5), a(5), alfa(5));

    % --- Paso 4: Matrices acumuladas ---
    A02 = A01*A12;
    A03 = A02*A23;
    A04 = A03*A34; % muñeca

    % --- Paso 5: Puntos de articulaciones ---
    P = [0 0 0;
         A01(1:3,4)';
         A02(1:3,4)';
         A03(1:3,4)';
         A04(1:3,4)'];

    % --- Paso 6: Configuracion del grafico ---
    figure(1); clf;
    hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis([-50 50 -50 50 0 50]); % ajustar segun dimensiones STL
    title('Brazo robotico con STL y cinematica directa');
    view(135,30);

    try
        % Base y eslabones iniciales
        dibujarSTL('BASE.stl', eye(4), [0.8 0.8 0.8]);
        dibujarSTL('LINK1.stl', A01, [0.9 0.5 0.3]);
        dibujarSTL('LINK2.stl', A02, [0.3 0.8 0.5]);
        dibujarSTL('LINK3.stl', A03, [0.4 0.6 0.9]);

        % Muneca + pinza (solo un STL)
        dibujarMunecaPinzaDebug('LINK4.stl', A04, q(4), q(5), [0.7 0.7 0.7]);

        camlight headlight;
        material dull;
        drawnow; % asegurar renderizado
    catch ME
        warning('%s: %s', ME.identifier, ME.message);
    end

    % --- Paso 7: Esqueleto ---
    plot3(P(:,1), P(:,2), P(:,3), 'k.-', 'LineWidth',2, 'MarkerSize',15);

    % --- Salida ---
    T = A04; % pose final de la muñeca
end

%% =================================================================
% Funcion auxiliar: dibujar STL normal
%% =================================================================
function dibujarSTL(nombreSTL, A, color)
    tri = stlread(nombreSTL);
    if isa(tri,'triangulation')
        fv.faces = tri.ConnectivityList;
        fv.vertices = tri.Points;
    else
        fv = tri;
    end
    % fv.vertices = fv.vertices * 0.1; % descomentar si necesitas escalar
    n = size(fv.vertices,1);
    homog = [fv.vertices, ones(n,1)] * A';
    fv.vertices = homog(:,1:3);

    patch('Faces', fv.faces, 'Vertices', fv.vertices, ...
        'FaceColor', color, 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);
end

%% =================================================================
% Funcion auxiliar: dibujar muñeca + pinza con debug
%% =================================================================
function dibujarMunecaPinzaDebug(nombreSTL, A_base, q_inclinacion, q_rotacion, color)
    tri = stlread(nombreSTL);
    if isa(tri,'triangulation')
        fv.faces = tri.ConnectivityList;
        fv.vertices = tri.Points;
    else
        fv = tri;
    end
    fv.vertices = fv.vertices * 0.1; % descomentar si es necesario

    % Centro de rotacion de la pinza dentro del STL
    c = mean(fv.vertices,1); % ajustar si es necesario

    % Mostrar rango de vertices antes de transformacion
    disp('Vertices min/max antes de transformacion:');
    disp([min(fv.vertices); max(fv.vertices)]);

    % Trasladar STL al origen local
    T1 = eye(4); T1(1:3,4) = -c;

    % Rotacion de la pinza sobre eje Z local
    theta = q_rotacion * pi/180;
    R_pinza = eye(4);
    R_pinza(1:3,1:3) = [cos(theta) -sin(theta) 0;
                         sin(theta)  cos(theta) 0;
                             0          0     1];

    % Volver al centro
    T2 = eye(4); T2(1:3,4) = c;

    % Inclinacion de la muñeca (sobre X local)
    theta_inc = q_inclinacion * pi/180;
    R_inclinacion = eye(4);
    R_inclinacion(1:3,1:3) = [1 0 0;
                               0 cos(theta_inc) -sin(theta_inc);
                               0 sin(theta_inc)  cos(theta_inc)];

    % Transformacion final
    A_final = A_base * R_inclinacion * T2 * R_pinza * T1;

    % Aplicar transformacion
    n = size(fv.vertices,1);
    homog = [fv.vertices, ones(n,1)] * A_final';
    fv.vertices = homog(:,1:3);

    % Mostrar rango de vertices despues de transformacion
    disp('Vertices min/max despues de transformacion:');
    disp([min(fv.vertices); max(fv.vertices)]);

    % Dibujar STL
    patch('Faces', fv.faces, 'Vertices', fv.vertices, ...
        'FaceColor', color, 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);
end
