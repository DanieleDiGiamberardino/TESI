%% SIMULAZIONE NEURALE - PUNTO SINGOLO (Point-to-Point)
clear all; clc; close all;

%% 1. CARICAMENTO RETE
% Assicurati che il file sia nella stessa cartella
if ~exist('Trained_Model_MPC.mat', 'file')
    error('File Trained_Model_MPC.mat non trovato! Esegui prima il training.');
end
load('Trained_Model_MPC.mat'); % Carica 'net', 'mu_X', 'sig_X'

%% 2. CONFIGURAZIONE SCENARIO
% Definisci dove parte e dove deve arrivare l'auto

% Start (Punto A)
start_x = -15; 
start_y = 20; 
start_theta = 0; % Orientata verso est

% Target (Punto B)
target_x = 30; 
target_y = 10;
% Nota: La rete cercherà di arrivare qui. 
% Se il dataset originale prevedeva l'arrivo con angolo 0, cercherà di allinearsi.

% Inizializzazione Stato
x_current = [start_x; start_y; start_theta; 0]; % [x, y, theta, phi]
L = 1.2;    % Passo veicolo
Ts = 0.1;   % Tempo di campionamento
u_prev = [0; 0]; 

% Array per salvare i dati
history_x = []; 
history_u = [];

fprintf('Avvio Simulazione Point-to-Point...\n');
fprintf('Start: (%.1f, %.1f) -> Target: (%.1f, %.1f)\n', start_x, start_y, target_x, target_y);

%% 3. CICLO DI SIMULAZIONE
max_steps = 1000;
tolerance = 0.3; % Distanza minima per considerare "Arrivato"

for t = 1:max_steps
    
    %% A. TRASFORMAZIONE RELATIVA (Il Trucco)
    % La rete sa guidare solo verso (0,0).
    % Spostiamo il mondo affinché il Target diventi l'origine (0,0).
    
    dx = x_current(1) - target_x;
    dy = x_current(2) - target_y;
    
    % Input che diamo alla rete:
    % 1. Distanza X relativa
    % 2. Distanza Y relativa
    % 3. Angolo attuale (Assoluto, perché la rete conosce il Nord/Est globale)
    % 4. Sterzo attuale
    % 5-6. Comandi precedenti
    
    % Normalizziamo l'angolo tra -pi e pi per sicurezza
    th_norm = atan2(sin(x_current(3)), cos(x_current(3)));
    
    input_vec = [dx; dy; th_norm; x_current(4); u_prev];
    
    %% B. PREDIZIONE RETE NEURALE
    % 1. Normalizzazione (Z-Score con le statistiche del training)
    input_norm = (input_vec' - mu_X) ./ sig_X;
    
    % 2. Predizione
    u_opt = predict(net, input_norm)';
    
    % 3. Saturazione di Sicurezza (Clamping)
    % Serve per evitare che la rete dia numeri folli se esce dal dominio conosciuto
    u_opt(1) = max(min(u_opt(1), 2.0), -2.0); % Max Velocità 2 m/s
    u_opt(2) = max(min(u_opt(2), 0.5), -0.5); % Max Sterzo 0.5 rad
    
    %% C. FISICA (Kinematic Model)
    dxdt = Car_Like_Model(x_current, u_opt, L);
    x_current = x_current + dxdt * Ts;
    
    % Aggiorniamo u_prev per il prossimo passo
    u_prev = u_opt;
    
    % Salvataggio storia
    history_x = [history_x; x_current'];
    history_u = [history_u; u_opt'];
    
    %% D. CHECK ARRIVO
    dist_to_target = sqrt((x_current(1)-target_x)^2 + (x_current(2)-target_y)^2);
    
    if dist_to_target < tolerance
        fprintf('Target Raggiunto al passo %d! Distanza residua: %.2fm\n', t, dist_to_target);
        break;
    end
end

if t == max_steps
    fprintf('Tempo scaduto! Non sono arrivato al target.\n');
end

%% 4. VISUALIZZAZIONE
figure(1); clf; set(gcf, 'Color', 'w', 'Name', 'Neural Point-to-Point');
hold on; grid on; axis equal;

% Disegno Start e Target
plot(start_x, start_y, 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
plot(target_x, target_y, 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Target');

% Disegno Traiettoria
plot(history_x(:,1), history_x(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Traiettoria NN');

% Disegno Orientamento (ogni 20 passi)
step_vis = 20;
quiver(history_x(1:step_vis:end,1), history_x(1:step_vis:end,2), ...
       cos(history_x(1:step_vis:end,3)), sin(history_x(1:step_vis:end,3)), ...
       0.5, 'Color', 'm', 'DisplayName', 'Orientamento');

legend('Location', 'best');
xlabel('X [m]'); ylabel('Y [m]');
title('Navigazione Neurale Punto-Punto');

% Grafico Comandi
figure(2); clf; set(gcf, 'Color', 'w', 'Name', 'Comandi Neurale');
time_axis = (0:length(history_u)-1) * Ts;

subplot(2,1,1);
plot(time_axis, history_u(:,1), 'b', 'LineWidth', 1.5);
yline(0, 'k--');
ylabel('Velocità [m/s]'); title('Controllo Longitudinale'); grid on;

subplot(2,1,2);
plot(time_axis, history_u(:,2), 'r', 'LineWidth', 1.5);
ylabel('Sterzo [rad]'); title('Controllo Laterale'); grid on; xlabel('Tempo [s]');