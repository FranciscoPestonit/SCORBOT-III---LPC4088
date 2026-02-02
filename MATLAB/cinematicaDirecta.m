function T = cinematicaDirecta(q)
    % Pasar q de grados a radianes
    q = q * pi/180;

    % Parámetros DH
    teta = [q(1) q(2)+pi/2+pi/6 -q(2)+q(3)-pi/2-pi/6 q(4)-q(3) q(5)];
    d = [35 0 0 0 14];
    alfa = [pi/2 0 0 pi/2 0];
    a = [2.5 22.5 22.5 0 0];

    % Matrices de transformación
    A01 = denavit(teta(1),d(1),a(1),alfa(1));
    A12 = denavit(teta(2),d(2),a(2),alfa(2));
    A23 = denavit(teta(3),d(3),a(3),alfa(3));
    A34 = denavit(teta(4),d(4),a(4),alfa(4));
    A45 = denavit(teta(5),d(5),a(5),alfa(5));

    A02 = A01*A12;
    A03 = A02*A23;
    A04 = A03*A34;
    A05 = A04*A45;
    T = A05;

    % Puntos de cada articulación
    P = [ 0 0 0;
          A01(1:3,4)';
          A02(1:3,4)';
          A03(1:3,4)';
          A04(1:3,4)';
          A05(1:3,4)' ];

    % === Configuración inicial del gráfico ===
    figure(1); clf;
    hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis([-80 80 -80 80 0 80]);
    title('Brazo robótico con geometría');
    view(135,30);

% === Dibujar base STL ===
try
    tri = stlread('BASE.stl');  % Cargar STL (devuelve triangulation)

    % Convertir a estructura compatible con patch()
    fv.faces = tri.ConnectivityList;
    fv.vertices = tri.Points;

    % Ajustar escala y posición (modifica si es necesario)
    escala = 0.1;
    desplazamiento = [0 0 0];  % [dx dy dz]
    fv.vertices = fv.vertices * escala + desplazamiento;

    % Dibujar STL
    patch('Faces', fv.faces, 'Vertices', fv.vertices, ...
        'FaceColor', [0.8 0.8 0.8], ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.15);

    camlight headlight;
    material dull;
catch ME
    warning('No se pudo cargar el archivo STL: %s', ME.message);
end


    % === Dibujar esqueleto del robot ===
    plot3(P(:,1),P(:,2),P(:,3),'k.-','LineWidth',2,'MarkerSize',15);

    % Dibujar cilindros entre articulaciones
    for i = 1:size(P,1)-1
        p1 = P(i,:);
        p2 = P(i+1,:);
        dibujarEslabon(p1,p2,2); % radio = 2 unidades
    end
end

function dibujarEslabon(p1,p2,r)
    % Dibuja un cilindro entre dos puntos 3D
    [X,Y,Z] = cylinder(r,20);
    L = norm(p2-p1);
    Z = Z*L;

    % Direcciones
    v = (p2-p1)/L;
    % Vector perpendicular arbitrario
    if abs(v(3)) < 0.99
        n = cross(v,[0 0 1]);
    else
        n = cross(v,[0 1 0]);
    end
    n = n/norm(n);
    b = cross(v,n);

    % Matriz de rotación
    R = [n; b; v];

    % Aplicar transformación
    P = R' * [X(:)'; Y(:)'; Z(:)'];
    Xr = reshape(P(1,:),size(X)) + p1(1);
    Yr = reshape(P(2,:),size(Y)) + p1(2);
    Zr = reshape(P(3,:),size(Z)) + p1(3);

    % Dibujar superficie
    surf(Xr,Yr,Zr,'FaceColor',[0.2 0.6 1],'EdgeColor','none');
end

function dibujarSTL(nombreSTL, A, color)
    % Cargar STL
    tri = stlread(nombreSTL);
    fv.faces = tri.ConnectivityList;
    fv.vertices = tri.Points;

    % Escala si tu modelo está en pulgadas → milímetros
    escala = 2.54;
    fv.vertices = fv.vertices * escala;

    % Aplicar transformación homogénea A (4x4)
    n = size(fv.vertices,1);
    homog = [fv.vertices, ones(n,1)] * A';  % aplicar transformación
    fv.vertices = homog(:,1:3);

    % Dibujar
    patch('Faces', fv.faces, 'Vertices', fv.vertices, ...
        'FaceColor', color, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.15);
end