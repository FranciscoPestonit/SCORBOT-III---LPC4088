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
    title('Brazo robótico con STL (solo bordes) y cinemática directa');
    view(135,30);

    % --- Paso 7: Dibujar STLs de cada parte (solo bordes) ---
    try
        dibujarSTLAristas('BASE.stl', eye(4), [0 0 0]);    % Negro
        dibujarSTLAristas('LINK1.stl', A01, [1 0 0]);      % Rojo
        dibujarSTLAristas('LINK2.stl', A02, [0 1 0]);      % Verde
        dibujarSTLAristas('LINK3.stl', A03, [0 0 1]);      % Azul
        dibujarSTLAristas('LINK4.stl', A04, [0.5 0.5 0]);  % Amarillo oscuro
        dibujarSTLAristas('LINK5.stl', A05, [1 0.5 0]);    % Naranja
    catch ME
        warning('Error al dibujar STLs: %s', ME.message);
    end

    % --- Paso 8: Esqueleto (opcional, para referencia) ---
    plot3(P(:,1),P(:,2),P(:,3),'k.-','LineWidth',2,'MarkerSize',15);
end

%% ================================================================
% Función auxiliar: Dibujar STL usando solo aristas
%% ================================================================
function dibujarSTLAristas(nombreSTL, A, color)
    tri = stlread(nombreSTL);

    % Adaptar según tipo de salida (estructura o triangulation)
    if isa(tri, 'triangulation')
        fv.faces = tri.ConnectivityList;
        fv.vertices = tri.Points;
    else
        fv = tri;
    end

    % Aplicar transformación homogénea
    n = size(fv.vertices,1);
    homog = [fv.vertices, ones(n,1)] * A';
    fv.vertices = homog(:,1:3);

    % Obtener aristas únicas
    edges = [fv.faces(:,[1 2]); fv.faces(:,[2 3]); fv.faces(:,[3 1])];
    edges = unique(sort(edges,2),'rows');

    % Dibujar cada arista
    for i = 1:size(edges,1)
        plot3(fv.vertices(edges(i,:),1), fv.vertices(edges(i,:),2), fv.vertices(edges(i,:),3), ...
              'Color', color, 'LineWidth', 1.5);
    end
end
