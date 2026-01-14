%% SIMULAZIONE CERCHIO CON RETE NEURALE (ROBUST + SOFT ALIGNED)
clear all; clc; close all;

%% 1. CARICAMENTO RETE
if ~exist('Trained_Model_MPC.mat', 'file')
    error('Devi prima eseguire Train_NN.m con il dataset SOFT!');
end
load('Trained_Model_MPC.mat'); 

%% 2. PARAMETRI STANDARD (ALLINEATI AL TRAINING SOFT)
% Questi limiti devono matchare quelli usati nel training per evitare comportamenti imprevisti
V_MAX_SYS = 2.5;   
W_MAX_SYS = 0.35;  
V_MIN_SYS = -0.5;  

% IMPORTANTE: Riduciamo questo valore per evitare l'effetto "taglio corda"
% Deve essere coerente con quello usato in Raccolta_Dati (es. 2.0 per Soft, 1.5 per Standard)
RAMP_DIST_TRAINING = 1.5; 

% Soglia di sicurezza (Input Clamping) - Come in PROVA_SINGO
MAX_VIEW_DIST_TRAINING = 15.0; 

%% 3. CONFIGURAZIONE SIMULAZIONE
R_circle = 10;
V_target_desiderata = 2.0; 

% Partenza sulla circonferenza
start_x = R_circle; start_y = 0; start_angle = pi/2;
x_current = [start_x; start_y; start_angle; 0]; 

L = 1.2; Ts = 0.1;
u_prev = [0; 0];
history_x = []; history_u = [];

loops = 0; prev_theta = atan2(start_y, start_x);

fprintf('Avvio Cerchio ROBUST (V_Target=%.1f, Ramp=%.1f)...\n', V_target_desiderata, RAMP_DIST_TRAINING);

for t = 1:2000

    %% A. CALCOLO TARGET DINAMICO
    theta_polare = atan2(x_current(2), x_current(1));

    % Gestione giri completi
    if (theta_polare - prev_theta) < -5, loops = loops + 1;
    elseif (theta_polare - prev_theta) > 5, loops = loops - 1; end
    prev_theta = theta_polare;
    current_arc = theta_polare + 2*pi*loops;

    % Look-ahead Ottimizzato (Per non tagliare la curva)
    % Stiamo APPENA fuori dalla rampa per mantenere la velocità
    min_look_ahead = RAMP_DIST_TRAINING + 0.1; 
    
    K_gain = 0.9; 
    look_ahead_dist = max(min_look_ahead, V_target_desiderata * K_gain);
    look_ahead_dist = min(look_ahead_dist, 3.0); % Saturazione stretta

    angle_target = current_arc + (look_ahead_dist / R_circle);
    xt = R_circle * cos(angle_target);
    yt = R_circle * sin(angle_target);
    
    %% B. ROBUSTEZZA & VIRTUAL TARGET (Il Fix di PROVA_SINGO)
    dx = x_current(1) - xt; 
    dy = x_current(2) - yt; 
    
    real_dist = sqrt(dx^2 + dy^2);
    
    % Se l'auto sbanda e si allontana troppo, scaliamo l'input per
    % riportarlo nel range che la rete conosce. Questo evita divergenze.
    if real_dist > MAX_VIEW_DIST_TRAINING
        scale = MAX_VIEW_DIST_TRAINING / real_dist;
        dx_input = dx * scale;
        dy_input = dy * scale;
    else
        dx_input = dx;
        dy_input = dy;
    end
    
    th_norm = atan2(sin(x_current(3)), cos(x_current(3))); 
    
    % Costruzione Input Esplicita
    input_vec = [dx_input; dy_input; th_norm; x_current(4); u_prev];

    %% C. PREDIZIONE
    input_norm = (input_vec' - mu_X) ./ sig_X;
    u_opt = predict(net, input_norm)';

    %% D. SATURAZIONE (Coerente col Training)
    u_opt(1) = max(min(u_opt(1), V_MAX_SYS), V_MIN_SYS); 
    u_opt(2) = max(min(u_opt(2), W_MAX_SYS), -W_MAX_SYS);

    %% E. FISICA
    dxdt = Car_Like_Model(x_current, u_opt, L);
    x_current = x_current + dxdt * Ts;
    u_prev = u_opt;
    
    history_x = [history_x; x_current'];
    history_u = [history_u; u_opt'];
end

%% VISUALIZZAZIONE
figure(1); clf; set(gcf,'Color','w', 'Name', 'Neural Circle Robust');
hold on; axis equal; grid on;
plot(R_circle*cos(0:0.01:2*pi), R_circle*sin(0:0.01:2*pi), 'k--', 'LineWidth', 1.0);
plot(history_x(:,1), history_x(:,2), 'b-', 'LineWidth', 2);
plot(start_x, start_y, 'go', 'MarkerSize', 8, 'LineWidth', 2);
title(['Guida Neurale (Robust Fix)']);
xlabel('X [m]'); ylabel('Y [m]');