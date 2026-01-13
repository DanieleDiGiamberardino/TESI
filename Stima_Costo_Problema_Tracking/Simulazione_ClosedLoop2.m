fprintf('Avvio Simulazione Closed-Loop con controllore\n');

% Carico le matrici (assumendo che i file siano nella cartella corrente)
load("Matrici_Stimate_TRACKING.mat");
load("Modello_Aumentato.mat");

% Ricavo dimensioni
s = size(Msy_stima, 1);
[~, m] = size(B_aum);
N = s / m;             % Orizzonte 
n_y = size(C_aum, 1);  % Numero uscite (1 per doppio int)
p = N * n_y;           % Dimensione vettore riferimento 
% Parametri Penalità
Q_y_vera = 1;      
R_delta_vera = 10; 

%% CALCOLO MATRICI REALI (Benchmark Model-Based)

S_bar = zeros(n_aum * N, m * N);
T_bar = zeros(n_aum * N, n_aum);

% Riempimento S e T (Evoluzione del sistema aumentato)
T_bk = eye(n_aum);
for k = 1:N
    T_bk = A_aum * T_bk;
    T_bar((k-1)*n_aum+1:k*n_aum, :) = T_bk;
    for j = 1:k
        if j==k, blk = B_aum; else, blk = (A_aum^(k-j))*B_aum; end
        S_bar((k-1)*n_aum+1:k*n_aum, (j-1)*m+1:j*m) = blk;
    end
end

C_bar = kron(eye(N), C_aum);

%  CORREZIONE Q_bar 
% Q_bar deve pesare l'errore (y - r). Poiché y e r sono vettori lunghi N (5),
% Q_bar deve essere N x N (oppure N*ny x N*ny). 
% NON usare eye(p) se p=5 dentro la kron, altrimenti esce 25x25!
Q_bar = kron(eye(N), Q_y_vera * eye(n_y)); 

R_bar = kron(eye(N), R_delta_vera * eye(m));

% Calcolo Matrici Vere
Msy_vera = R_bar + (C_bar*S_bar)' * Q_bar * (C_bar*S_bar);
Hsy_vera = (C_bar*S_bar)' * Q_bar * (C_bar*T_bar);

% CORREZIONE vu_vera 
% Questo termine lega l'input DeltaU al riferimento r.
% Matematicamente corrisponde a: -(C_bar*S_bar)' * Q_bar
% Dimensione attesa: s x p (5x5)
vu_vera = (C_bar * S_bar)' * Q_bar;

%% SIMULAZIONE
T_sim = 100;           % Durata simulazione 
x0 = [0; 0];           % Stato iniziale 
u_prev = 0;            % Controllo precedente iniziale
Ampiezza = 1.0;
Omega = 0.2; 
% Inizializzo vettori storico STIMA
y_hist_stima = zeros(1, T_sim);
u_hist_stima = zeros(1, T_sim);
x_aum_init = [x0; u_prev];

% Inizializzo vettori storico VERO
y_hist_vera = zeros(1, T_sim);
u_hist_vera = zeros(1, T_sim);
r_hist = zeros(1, T_sim); % Per plottare il riferimento

x_tilde_stima = x_aum_init;
x_tilde_vera  = x_aum_init;

for k = 1:T_sim
    % VETTORE DI RIFERIMENTO
    steps_future = (0:p-1)'; 
    r_vec = Ampiezza * sin(Omega * (k + steps_future));
    
    % Salvo il valore corrente del riferimento per il grafico
    r_hist(k) = r_vec(1);
    
    % CONTROLLO DATA-DRIVEN 
    % Nota: vu_stima * r_vec (Matrice * Vettore)
    DeltaU_stima_ott = Msy_stima \ ((vu_stima * r_vec) - (Hsy_stima * x_tilde_stima));
    
    deltau_stima = DeltaU_stima_ott(1:m); 
    u_curr_stima = x_tilde_stima(end) + deltau_stima; 
    
    y_hist_stima(k) = C_aum * x_tilde_stima;
    u_hist_stima(k) = u_curr_stima;
    x_tilde_stima = A_aum * x_tilde_stima + B_aum * deltau_stima;
    
    % --- CONTROLLO MODEL-BASED (VERO) ---
    DeltaU_vera_ott = Msy_vera \ ((vu_vera * r_vec) - (Hsy_vera * x_tilde_vera));
    
    deltau_vera = DeltaU_vera_ott(1:m); 
    u_curr_vera = x_tilde_vera(end) + deltau_vera; 
    
    y_hist_vera(k) = C_aum * x_tilde_vera;
    u_hist_vera(k) = u_curr_vera;
    x_tilde_vera = A_aum * x_tilde_vera + B_aum * deltau_vera;
end
%% GRAFICI
t_axis = 0:T_sim-1;
figure('Color', 'w', 'Name', 'Confronto Finale (Discreto)', 'Position', [100, 100, 1000, 500]);

% GRAFICO U(k)
% Stairs è perfetto per mostrare che l'input è costante tra k e k+1
subplot(1, 2, 1);
stairs(t_axis, u_hist_vera, 'b', 'LineWidth', 2); hold on;
stairs(t_axis, u_hist_stima, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time steps (k)');
ylabel('Amplitude');
title('Control Input $$u(k)$$', 'Interpreter', 'latex', 'FontSize', 16);
legend('MPC Vero', 'Data-Driven');

% GRAFICO Y(k)
% Uso stairs anche qui per mostrare la natura discreta dell'uscita campionata
subplot(1, 2, 2);
stairs(t_axis, y_hist_vera, 'b', 'LineWidth', 2); hold on;
stairs(t_axis, y_hist_stima, 'r--', 'LineWidth', 2);
stairs(t_axis, r_hist, 'k:', 'LineWidth', 2); % Anche il riferimento a gradini
grid on;
xlabel('Time steps (k)');
ylabel('Amplitude');
title('Tracking Output $$y(k)$$', 'Interpreter', 'latex', 'FontSize', 16);
legend('MPC Vero', 'Data-Driven', 'Riferimento');
ylim([min(r_hist)-0.5, max(r_hist)+0.5]);