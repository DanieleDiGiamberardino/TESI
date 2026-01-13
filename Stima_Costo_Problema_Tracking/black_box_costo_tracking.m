function J_track = black_box_costo_tracking(x0,u_prev,delta_U_seq,r_ref,A_aum,B_aum,C_aum,Q_y_vera,R_delta_vera)
% INPUT:
% x0, u_prev:    Stato iniziale e controllo precedente x_aum(0)
% Delta_U_seq:   Sequenza di controllo [du_0; du_1; ...] ( è la z)
% r_ref:         Il riferimento da inseguire
% A_aum, B_aum:  Matrici del modello aumentato
% Q_y_vera:      Matrice di costo VERA che moltiplica l'errore di tracking (y-r)^2
% R_delta_vera:  Matrice di costo VERA che moltiplica l'incremento (Delta_u)^2

[~, m] = size(B_aum); % Dimensione input
s = length(delta_U_seq); % Lunghezza tot del vettore di incremento
N = s / m; % Orizzonte N

Delta_U_matrix = reshape(delta_U_seq, m, N); % Uso una matrice più semplice

% Definisco lo stato iniziale aumentato
x_aum_k = [x0; u_prev];
J_track = 0; % Costo totale iniziale

% Ciclo di simulazione ( uso come indice di costo non quello in forma
% quadratica) (slide 52 in alto )
for k = 1:N
    delta_u_k = Delta_U_matrix(:, k); % Definisco l'incremento di controllo passo k

    % Calcolo lo stato successivo x(k+1)
    x_aum_k_next = A_aum * x_aum_k + B_aum * delta_u_k;

    % Calcolo l'uscita successiva
    y_k_next = C_aum * x_aum_k_next;

    % Calcolo l'errore di tracking
    error_tracking = y_k_next - r_ref(k);

    % Calcolo costo dell'incremento u(k)
    costo_u = delta_u_k' * R_delta_vera * delta_u_k;

    % Calcolo costo dell'errore
    costo_err = error_tracking' * Q_y_vera * error_tracking;

    % Calcolo costo effettivo fino al k passo
    J_track = J_track + costo_err + costo_u;

    x_aum_k = x_aum_k_next;
end
end