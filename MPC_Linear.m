function [u_applied, delta_u_applied] = MPC_Linear(x0, u_prev, y_ref, N, Ts, L, Q, R, u_min, u_max, du_min, du_max, C)
% MPC LINEARE
n_x = 4; n_u = 2; n_y = size(C, 1);

% PUNTO DI LINEARIZZAZIONE
x_lin = x0;
u_lin = u_prev;

%  FIX SINGOLARITÀ
% Problema: Se la velocità è 0, la matrice B perde controllabilità
% sullo sterzo (l'auto pensa che girare le ruote non faccia nulla).
% Soluzione: Calcoliamo lo Jacobiano usando una velocità "virtuale" minima
% solo per la costruzione della matrice, preservando il segno.

v_thresh = 0.1; % Soglia
v_curr = u_lin(1);

if abs(v_curr) < v_thresh
    % Mantiene il segno ma assicura modulo minimo.
    % Se è proprio 0, assume positivo (partenza).
    if v_curr >= 0
        u_lin_model = [max(v_curr, 0.5); u_lin(2)];
    else
        u_lin_model = [min(v_curr, -0.5); u_lin(2)];
    end
else
    u_lin_model = u_lin;
end

% Chiamata al modello linearizzato con input "sicuro"
[A, B, d] = Linearized_Model(x_lin, u_lin_model, Ts, L); % Calcolo d usando u_fitizzia e questo rende coerente il tutto

% Aumentiamo lo stato per includere l'integratore dei delta input: xi = [x; u_prev]
A_aum = [A, B; zeros(n_u, n_x), eye(n_u)];
B_aum = [B; eye(n_u)];

% Il termine noto aumentato deve includere la dinamica affine
d_aum = [d; zeros(n_u, 1)];

C_aum = [C, zeros(n_y, n_u)];

zi_0 = [x0; u_prev]; % Stato iniziale aumentato

%% Costruisco le matrici S, T, Dc
S = zeros(N*n_y, N*n_u);
T = zeros(N*n_y, 6);
D_c = zeros(N*n_y, 1);

A_pow = eye(6);
d_acc = zeros(6, 1);

for k = 1:N
    % Accumulo termine noto (d_acc rappresenta l'evoluzione libera del termine affine)
    if k == 1
        d_acc = d_aum;
        A_pow = A_aum;
    else
        d_acc = A_aum * d_acc + d_aum;
        A_pow = A_aum * A_pow;
    end

    idx_row = (k-1)*n_y + 1 : k*n_y;

    % Risposta libera dello stato iniziale
    T(idx_row, :) = C_aum * A_pow;

    % Risposta libera del termine affine
    D_c(idx_row, 1) = C_aum * d_acc;

    % Matrice di convoluzione (Risposta forzata)
    for j = 1:k
        if j == k
            blk = C_aum * B_aum;
        else
            blk = C_aum * (A_aum^(k-j)) * B_aum;
        end
        idx_col = (j-1)*n_u + 1 : j*n_u;
        S(idx_row, idx_col) = blk;
    end
end

%% COSTRUZIONE FUNZIONE COSTO
Q_bar = kron(eye(N), Q);
R_bar = kron(eye(N), R);

% Riferimento: y_ref è un vettore colonna lungo 4*N
Y_ref = y_ref;
if length(Y_ref) ~= N*n_y
    error('Dimensione y_ref non coerente con N');
end

% H = S' * Q_bar * S + R_bar
H = 2 * (S' * Q_bar * S + R_bar);
H = (H + H') / 2; % Simmetria numerica

f = 2 * S' * Q_bar * ((T * zi_0 + D_c) - Y_ref);

%% VINCOLI
% Vincoli su Delta U (LB, UB)
LB = repmat(du_min, N, 1);
UB = repmat(du_max, N, 1);

% Vincoli su U assoluto (trasformati in vincoli su Delta U)

M_tril = kron(tril(ones(N)), eye(n_u));

U_prev_vec = repmat(u_prev, N, 1);
U_max_vec = repmat(u_max, N, 1);
U_min_vec = repmat(u_min, N, 1);

% A_ineq * Delta_U <= b_ineq
%  M * dU <= U_max - U_prev
% -M * dU <= U_prev - U_min
A_ineq = [ M_tril; -M_tril];
b_ineq = [ U_max_vec - U_prev_vec; U_prev_vec - U_min_vec];

% RISOLUZIONE
options = optimoptions('quadprog', 'Display', 'off');
[Delta_U_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], LB, UB, [], options);

if exitflag == 1
    delta_u_applied = Delta_U_opt(1:n_u);
    u_applied = u_prev + delta_u_applied;
else
    % Fallback
    delta_u_applied = [0;0];
    u_applied = u_prev;
end
end