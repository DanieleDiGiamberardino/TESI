load("Script_raccolta_DatiCosto_2.mat");

% Risolvo Y = Phi * p per il vettore p

% p = [vec(M); vec(H); vec(Q)]

p_stimato = pinv(Phi) * Y_costo;

s = m * N;
num_parametri_M = s * s;
num_parametri_H = s * n;
num_parametri_Qb = n * n;

% Ottengo i vettori M,H,Q
vec_M_stima = p_stimato(1 : num_parametri_M);
vec_H_stima = p_stimato(num_parametri_M + 1 : num_parametri_M + num_parametri_H);
vec_Qb_stima = p_stimato(num_parametri_M + num_parametri_H + 1 : end);

% Riforma le matrici
M_stima = reshape(vec_M_stima, s, s);
H_stima = reshape(vec_H_stima, s, n);
Qb_stima = reshape(vec_Qb_stima, n, n);

% Verifica (mi devo costruire le matrici M , H e Q_b)
A_v = [1 1;
    0 1];
B_v = [0; 1];
Q_sv = [0.2 0;
    0 0];
R_v = 1;
S_v = Q_sv;

% Costruisco le matrici "batch"
S_bar = zeros(n*N, m*N);
T_bar = zeros(n*N, n);
Q_bar = zeros(n*N, n*N);
R_bar = zeros(m*N, m*N);

T_bar(1:n, :) = A_v;
S_bar(1:n, 1:m) = B_v;

% Costruisco le matrici Y, F^T e H (slide 14)

% Nel for riempio le matrici T_bar e S_bar
for i = 2:N
    idx_row = (i-1)*n + 1 : i*n;
    idx_row_prev = (i-2)*n + 1 : (i-1)*n;

    T_bar(idx_row, :) = A_v * T_bar(idx_row_prev, :); % qui sfrutto il fatto di usare  le righe precedenti cosi 
    % moltiplico solo per A_v 

    S_bar(idx_row, 1:m) = A_v * S_bar(idx_row_prev, 1:m);

    for j = 2:i

        idx_col_prev = (j-2)*m + 1 : (j-1)*m;
        idx_col = (j-1)*m + 1 : j*m;

        S_bar(idx_row, idx_col) = S_bar(idx_row_prev, idx_col_prev);
    end
end
% Rendo la matrice triangolare inferiore
S_bar = tril(S_bar);

% Riempimento Q_bar e R_bar (Matrici di penalizzazione dell intera finistra
% N
for i = 1:N
    idx_n = (i-1)*n + 1 : i*n;
    idx_m = (i-1)*m + 1 : i*m;

    if i < N
        Q_bar(idx_n, idx_n) = Q_sv;
    else
        Q_bar(idx_n, idx_n) = S_v; % Costo terminale all'ultimo posto
    end
    R_bar(idx_m, idx_m) = R_v;
end

% Calcolo le matrici batch "vere" M, H, Qb

% Formula del PDF ( slide 14 in basso)
% J = U'*(R_bar + S_bar'*Q_bar*S_bar)*U + 2*U'*(S_bar'*Q_bar*T_bar)*x0 + x0'*(Q_stage_vera + T_bar'*Q_bar*T_bar)*x0

M_vera = R_bar + S_bar' * Q_bar * S_bar; % La F_slide (devo trasporre quella della slide)
H_vera = S_bar' * Q_bar * T_bar; % La H_slide
Qb_vera = Q_sv + T_bar' * Q_bar * T_bar; % La Y_slide

% CONFRONTO I RISULTATI
err_M = norm(M_vera - M_stima) / norm(M_vera);
err_H = norm(H_vera - H_stima) / norm(H_vera);
err_Qb = norm(Qb_vera - Qb_stima) / norm(Qb_vera);

fprintf('\n--- VERIFICA STIMA ---\n');
fprintf('Errore relativo M_stima: %.4f %%\n', err_M * 100);
fprintf('Errore relativo H_stima: %.4f %%\n', err_H * 100);
fprintf('Errore relativo Qb_stima: %.4f %%\n', err_Qb * 100);

disp('Qb Vera:'); disp(Qb_vera);
disp('Qb Stimata:'); disp(Qb_stima);

%Salvo i dati 

save("Stima_costo_J.mat","Qb_stima","H_stima","M_stima","M_vera","H_vera","Qb_vera");





