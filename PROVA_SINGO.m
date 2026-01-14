%% SIMULAZIONE NEURALE - PUNTO SINGOLO (CON VIRTUAL TARGET FIX)
clear all; clc; close all;

%% 1. CARICAMENTO
if ~exist('Trained_Model_MPC.mat', 'file'), error('File mancante'); end
load('Trained_Model_MPC.mat'); 

%% 2. PARAMETRI COERENTI
V_MAX_SYS = 2.5;   
W_MAX_SYS = 0.35;  
V_MIN_SYS = -0.5; 

%% 3. SCENARIO DIFFICILE (Quello che ti dava problemi)
start_x = -15; start_y = 300; start_theta = 0; 
target_x = 45; target_y = 10; % Distanza enorme (~46m)

x_current = [start_x; start_y; start_theta; 0]; 
L = 1.2; Ts = 0.1; u_prev = [0; 0]; 
history_x = []; history_u = [];

fprintf('Start: (%.1f, %.1f) -> Target: (%.1f, %.1f)\n', start_x, start_y, target_x, target_y);

% Distanza massima che la rete "conosce" bene (dal training)
MAX_VIEW_DIST = 15.0; 

for t = 1:2000 % Aumentiamo step perché il viaggio è lungo
    
    %% A. TRASFORMAZIONE & INPUT CLAMPING (Il Fix Fondamentale)
    dx = x_current(1) - target_x;
    dy = x_current(2) - target_y;
    
    real_dist = sqrt(dx^2 + dy^2);
    
    % --- VIRTUAL TARGET LOGIC ---
    % Se siamo lontani 45m, la rete impazzisce perché ha visto max 25m nel training.
    % Quindi "inganniamo" la rete: le diciamo che il target è a MAX_VIEW_DIST
    % nella stessa direzione.
    
    if real_dist > MAX_VIEW_DIST
        scale_factor = MAX_VIEW_DIST / real_dist;
        dx_input = dx * scale_factor;
        dy_input = dy * scale_factor;
    else
        dx_input = dx;
        dy_input = dy;
    end
    
    % Normalizzazione angolo
    th_norm = atan2(sin(x_current(3)), cos(x_current(3)));
    
    % Input con dx, dy CLAMPATI
    input_vec = [dx_input; dy_input; th_norm; x_current(4); u_prev];
    
    %% B. PREDIZIONE
    input_norm = (input_vec' - mu_X) ./ sig_X;
    u_opt = predict(net, input_norm)';
    
    % Saturazione
    u_opt(1) = max(min(u_opt(1), V_MAX_SYS), V_MIN_SYS); 
    u_opt(2) = max(min(u_opt(2), W_MAX_SYS), -W_MAX_SYS); 
    
    %% C. FISICA
    dxdt = Car_Like_Model(x_current, u_opt, L);
    x_current = x_current + dxdt * Ts;
    u_prev = u_opt;
    
    history_x = [history_x; x_current'];
    history_u = [history_u; u_opt'];
    
    if real_dist < 0.3
        fprintf('Arrivato al passo %d!\n', t); break;
    end
end

%% VISUALIZZAZIONE
figure(1); clf; set(gcf, 'Color', 'w'); hold on; axis equal; grid on;
plot(start_x, start_y, 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(target_x, target_y, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
plot(history_x(:,1), history_x(:,2), 'b-', 'LineWidth', 2);
title('Navigazione con Virtual Target (No Divergenza)');
xlabel('X [m]'); ylabel('Y [m]');