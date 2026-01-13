fprintf("\nVerifica di ottimalità Controllo U\n");

load("Script_raccolta_DatiCosto_2.mat","n","m","N");
load("Stima_costo_J.mat");

% Carico i parametri "veri"
A_v = [1 1;
    0 1];
B_v = [0; 1];
Q_sv = [0.2 0;
    0 0];
R_v = 1;
S_v = Q_sv;

% In Stima_costo mi sono salvato le matrici M_vera e H_vera

T_sim = 50; % Passi simulazione
x0 = [5; 0]; % Stato iniziale
noise = 0.01;

% Inizializzo Vettori di Raccolta
x_hist_stima = zeros(n,T_sim +1);
u_hist_stima = zeros(m,T_sim);

x_hist_vera = zeros(n,T_sim +1);
u_hist_vera = zeros(m,T_sim);

% Inizializzo vettori (lo stato iniziale deve essere uguale)
x_hist_vera(:,1) = x0;
x_hist_stima(:,1) = x0;

% Stati correnti
x_curr_stima = x0;
x_curr_vera  = x0;

% Definisco le variabili per J_history
J_cum_stima = 0;
J_cum_vera = 0;

for k = 1:T_sim
    % Genero il disturbo
    d = noise * randn(n , 1);

    % Calcolo la sequenza di controllo poi prendo solo il primo valore
    U_seq_stima = -(M_stima \ H_stima) * x_curr_stima;
    u_stima = U_seq_stima(1:m);

    % Definisco costo attuale e costruisco quello comulativo
    costo_att_stima = x_curr_stima' * Q_sv * x_curr_stima + u_stima' * R_v * u_stima;
    J_cum_stima = J_cum_stima + costo_att_stima;

    x_next_stima = A_v * x_curr_stima + B_v * u_stima + d; % Ho aggiunto direttamente qui il distrubo

    % Mi salvo la storia stimata
    u_hist_stima(:,k) = u_stima;
    x_hist_stima(:,k+1) = x_next_stima;
    x_curr_stima = x_next_stima; % Aggiorna stato

    % Calcolo la sequenza di controllo ottima reale
    U_seq_vera = -(M_vera \ H_vera) * x_curr_vera;
    u_vera = U_seq_vera(1:m);

    % Definisco costo attuale e costruisco quello comulativo
    costo_att_vera = x_curr_vera' * Q_sv * x_curr_vera + u_vera' * R_v * u_vera;
    J_cum_vera = J_cum_vera  + costo_att_vera;

    x_next_vera = A_v * x_curr_vera + B_v * u_vera + d ;
    
    % Mi salvo la storia vera
    u_hist_vera(:,k) = u_vera;
    x_hist_vera(:,k+1) = x_next_vera;
    x_curr_vera = x_next_vera; % Aggiorna stato
end

% Aggiungo al costo Totale anche il costo terminale
J_cum_stima = J_cum_stima + x_curr_stima' * S_v * x_curr_stima;
J_cum_vera  = J_cum_vera + x_curr_vera' * S_v * x_curr_vera;

loss_perc = ((J_cum_stima - J_cum_vera) / J_cum_vera) * 100;

fprintf('\n--- RISULTATI PERFORMANCE ---\n');
fprintf('Costo Cumulativo VERO (Benchmark): %.4f\n', J_cum_vera);
fprintf('Costo Cumulativo STIMATO:          %.4f\n', J_cum_stima);
fprintf('Perdita di Performance:            %.2f%%\n', loss_perc);

% --- 6. GRAFICI DEI RISULTATI ---
t_axis = 0:T_sim;
t_axis_u = 0:T_sim-1;
figure('Name', 'Confronto Closed-Loop: Stimato vs Vero', 'Color', 'w');

% Plot Stato 1 (Posizione)
subplot(3,1,1);
plot(t_axis, x_hist_vera(1,:), 'b', 'LineWidth', 2, 'DisplayName', 'Benchmark (Vero)');
hold on;
plot(t_axis, x_hist_stima(1,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Data-Driven (Stimato)');
ylabel('X_1'); grid on; legend;
title('Confronto Traiettorie a Ciclo Chiuso (con Disturbo)');

% Plot Stato 2 (Velocità)
subplot(3,1,2);
plot(t_axis, x_hist_vera(2,:), 'b', 'LineWidth', 2);
hold on;
plot(t_axis, x_hist_stima(2,:), 'r--', 'LineWidth', 2);
ylabel('X_2'); grid on;

% Plot Ingresso (Controllo)
subplot(3,1,3);
stairs(t_axis_u, u_hist_vera(1,:), 'b', 'LineWidth', 2);
hold on;
stairs(t_axis_u, u_hist_stima(1,:), 'r--', 'LineWidth', 2);
ylabel('Input u'); xlabel('Tempo k'); grid on;