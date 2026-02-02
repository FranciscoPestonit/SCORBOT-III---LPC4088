function q=cinematicaInversa(p)

x = p(1);
y = p(2);
z = p(3);

% Se pasan los angulos de grados a rad
q4 = p(4)*pi/180;

% Parametros DH
d = [35 0 0 0 14];
alfa = [pi/2 0 0 pi/2 0];
a = [2.5 22.5 22.5 0 0];

% Cálculo de R, z13 y r
R = sqrt(x^2+y^2)-d(5)*sin(q4)-a(1);
z13 = z + d(5)*cos(q4)-d(1);
r = sqrt(R^2+z13^2);

% Cálculo de q1
q(1)=atan2(y, x);

% Cálculo de q2
cbeta = (r^2+a(2)^2-a(3)^2)/(2*r*a(2));
sbeta = sqrt(1-cbeta^2);
calfa = (z13)/r;
salfa = R/r;
beta = atan2(sbeta,cbeta);
alfa = atan2(salfa,calfa);
q(2) = alfa - beta + pi/6;

% Cálculo de q3
cq3 = (R-a(2)*sin(q(2)-pi/6))/a(3);
sq3 = (z13-a(3)*cos(q(2)-pi/6))/a(3);
q(3) = atan2(sq3,cq3);

%Tengo que cambiar q3 de signo

q(2) = -q(2);

% Se pasan los ángulos de radines a grados
q = q*180/pi;

% q4 y q5 coinciden con p(4) y p(5)
q(4) = p(4);
q(5) = p(5);

end