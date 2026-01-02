function [A, B, d] = Linearized_Model(x_bar, u_bar, Ts, L)
% Calcolo il modello linearizzato discreto: x(k+1) = A*x(k) + B*u(k) + d
% Linearizzato intorno al punto di lavoro (x_bar, u_bar)

% Estrazione variabili per leggibilità
theta = x_bar(3);
phi   = x_bar(4);
v     = u_bar(1);

% CALCOLO JACOBIANI CONTINUI

% J_x (Matrice 4x4)
J_x = zeros(4,4);
J_x(1, 3) = -v * sin(theta);
J_x(2, 3) =  v * cos(theta);
J_x(3, 4) = (v / L) * sec(phi)^2;

% J_u (Matrice 4x2)
J_u = zeros(4,2);
J_u(1, 1) = cos(theta);
J_u(2, 1) = sin(theta);
J_u(3, 1) = tan(phi) / L;
J_u(4, 2) = 1;

% DISCRETIZZAZIONE
% x(k+1) = (I + Ts*Jx)*x + (Ts*Ju)*u

A = eye(4) + Ts * J_x;
B = Ts * J_u;

% CALCOLO TERMINE AFFINE
% d = Ts * f(x_bar, u_bar) - Ts * Jx * x_bar - Ts * Ju * u_bar

% Calcolo f(x_bar, u_bar)
f_bar = Car_Like_Model(x_bar, u_bar, L);

% Termine d finale
d = Ts * (f_bar - J_x * x_bar -  J_u * u_bar);

end