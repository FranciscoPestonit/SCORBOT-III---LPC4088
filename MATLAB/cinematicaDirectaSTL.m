function T = cinematicaDirectaSTL(q)
    % === Cinemática directa y visualización 3D con STL ===
    % q: vector de ángulos articulares [q1 q2 q3 q4 q5] en grados
    % Devuelve la matriz homogénea final T

    % --- Paso 1: Pasar q a radianes ---
    q = q * pi/180;

    % --- Paso 2: Parámetros DH ---
    teta = [q(1) q(2)+pi/2+pi/6 -q(2)+q(3)-pi/2-pi/6 q(4)-q(3) q(5)];
    d = [35 0 0 0 14];
    alfa = [pi/2 0 0 pi/2 0];
    a = [1.5 22.2 22.2 0 0];

    % --- Paso 3: Matrices homogéneas individuales ---
    A01 = denavit(teta(1),d(1),a(1),alfa(1));
    A12 = denavit(teta(2),d(2),a(2),alfa(2));
    A23 = denavit(teta(3),d(3),a(3),alfa(3));
    A34 = denavit(teta(4),d(4),a(4),alfa(4));
    A45 = denavit(teta(5),d(5),a(5),alfa(5));

    % --- Paso 4: Acumuladas ---
    A02 = A01*A12;
    A03 = A02*A23;
    A04 = A03*A34;
    A05 = A04*A45;
    T = A05;
    % Esta matriz es para rotar el STL LINK4
    R_q5 = [ cos(q(5)) -sin(q(5)) 0 0;
         sin(q(5))  cos(q(5)) 0 0;
               0         0  1 0;
               0         0  0 1];
    % --- Paso 5: Puntos de articulaciones (para referencia visual) ---
    P = [ 0 0 0;
          A01(1:3,4)';
          A02(1:3,4)';
          A03(1:3,4)';
          A04(1:3,4)';
          A05(1:3,4)' ];

    % --- Paso 6: Configuración del gráfico ---
    figure(1); clf;
    hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis([-100 100 -100 100 0 100]);
    title('Brazo robótico con STL y cinemática directa');
    view(135,30);

    % --- Paso 7: Dibujar STLs de cada parte ---
    try
        % Base fija
        dibujarSTL('BASE.stl', eye(4), [0.8 0.8 0.8]);

        % Eslabones (colores distintos)
        dibujarSTL('LINK1.stl', A01, [0.9 0.5 0.3]); %Giro de la base
        dibujarSTL('LINK2.stl', A02, [0.3 0.8 0.5]); %Giro del hombro
        dibujarSTL('LINK3.stl', A03, [0.4 0.6 0.9]); %Giro del codo
        dibujarSTL('LINK4.stl', A04*R_q5, [0.7 0.7 0.7]); %Inclinación MÁS Rotación de la muñeca
%        dibujarSTL('LINK5.stl', A05, [1 0.8 0.2]);

        camlight headlight;
        material dull;
    catch ME
        warning(ME.identifier, 'Error al dibujar STLs: %s', ME.message);
    end

    % --- Paso 8: Esqueleto (opcional, para referencia) ---
    plot3(P(:,1),P(:,2),P(:,3),'k.-','LineWidth',2,'MarkerSize',15);
end


%% ================================================================
%  Función auxiliar: Dibujar STL con transformación homogénea
% ================================================================
function dibujarSTL(nombreSTL, A, color)
    % Cargar el STL
    tri = stlread(nombreSTL);

    % Adaptar según tipo de salida (estructura o triangulation)
    if isa(tri, 'triangulation')
        fv.faces = tri.ConnectivityList;
        fv.vertices = tri.Points;
    else
        fv = tri;
    end

    % Convertir mm a cm
    fv.vertices = fv.vertices * 0.1;

    % Aplicar la transformación homogénea A (4x4)
    n = size(fv.vertices,1);
    homog = [fv.vertices, ones(n,1)] * A';
    fv.vertices = homog(:,1:3);

    % Dibujar
    patch('Faces', fv.faces, 'Vertices', fv.vertices, ...
        'FaceColor', color, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.15);
end