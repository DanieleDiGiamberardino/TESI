function dxdt = Car_Like_Model(state, u, L)
% MODELLO CINEMATICO BICICLETTA (Car-Like)

% STATI (4x1):
% x:     Posizione X
% y:     Posizione Y
% theta: Angolo di imbardata (orientamento)
% phi:   Angolo di sterzo (ruote anteriori)

% INPUT (2x1):

% v:     Velocità longitudinale (acceleratore/freno)
% w:     Velocità di sterzata

% PARAMETRO:
% L:     Distanza ruote davanti-dietro

% Estrazione variabili
theta = state(3);
phi   = state(4);

% Variabili di Controllo
v = u(1);
w = u(2);

% Modello
dx     = v * cos(theta);      % Velocità su X
dy     = v * sin(theta);      % Velocità su Y
dtheta = (v / L) * tan(phi);  % Velocità angolare (curvatura)
dphi   = w;                   % Velocità cambio sterzo

dxdt = [dx; dy; dtheta; dphi];
end