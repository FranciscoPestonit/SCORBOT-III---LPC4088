function ejes_transformados
% Define matriz de rotación 3x3 (ejemplo: rotación de 45° alrededor de Z)
theta = pi/4;
R = [cos(theta) -sin(theta) 0;
     sin(theta)  cos(theta) 0;
     0           0          1];

% Origen del sistema original y del transformado
O = [0;0;0];
Ot = [0; 0; 0];  % ejemplo: origen desplazado (si se desea)

% Ejes del sistema original (columnas: X, Y, Z)
E = eye(3);

% Ejes transformados
Et = R * E;

% Escala para visualizar
s = 0.4;

figure; hold on; grid on; axis equal;
% Ejes originales en O
quiver3(O(1),O(2),O(3), s*E(1,1), s*E(2,1), s*E(3,1), 'r', 'LineWidth',2,'MaxHeadSize',0.5);
quiver3(O(1),O(2),O(3), s*E(1,2), s*E(2,2), s*E(3,2), 'g', 'LineWidth',2,'MaxHeadSize',0.5);
quiver3(O(1),O(2),O(3), s*E(1,3), s*E(2,3), s*E(3,3), 'b', 'LineWidth',2,'MaxHeadSize',0.5);

% Ejes transformados en Ot
quiver3(Ot(1),Ot(2),Ot(3), s*Et(1,1), s*Et(2,1), s*Et(3,1), 'r--', 'LineWidth',2,'MaxHeadSize',0.5);
quiver3(Ot(1),Ot(2),Ot(3), s*Et(1,2), s*Et(2,2), s*Et(3,2), 'g--', 'LineWidth',2,'MaxHeadSize',0.5);
quiver3(Ot(1),Ot(2),Ot(3), s*Et(1,3), s*Et(2,3), s*Et(3,3), 'b--', 'LineWidth',2,'MaxHeadSize',0.5);

% Etiquetas
text(O(1)+s*E(1,1), O(2)+s*E(2,1), O(3)+s*E(3,1), 'X');
text(O(1)+s*E(1,2), O(2)+s*E(2,2), O(3)+s*E(3,2), 'Y');
text(O(1)+s*E(1,3), O(2)+s*E(2,3), O(3)+s*E(3,3), 'Z');

text(Ot(1)+s*Et(1,1), Ot(2)+s*Et(2,1), Ot(3)+s*Et(3,1), 'X_{rot}');
text(Ot(1)+s*Et(1,2), Ot(2)+s*Et(2,2), Ot(3)+s*Et(3,2), 'Y_{rot}');
text(Ot(1)+s*Et(1,3), Ot(2)+s*Et(2,3), Ot(3)+s*Et(3,3), 'Z_{rot}');

xlabel('X'); ylabel('Y'); zlabel('Z');
title('Sistema original (sólido) y sistema rotado (guión)');
view(3);

end