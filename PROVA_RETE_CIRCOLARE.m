%% SIMULAZIONE CERCHIO CON RETE NEURALE (APPROCCIO PURO)
clear all; clc; close all;

%% CARICAMENTO RETE
if ~exist('Trained_Model_MPC.mat', 'file')
    error('Devi prima eseguire Train_NN.m!');
end
load('Trained_Model_MPC.mat'); 

%% CONFIGURAZIONE
R_circle = 10;
V_target = 1.5; % Velocità desiderata (usata solo per calibrare il look-ahead)

start_x = 20; start_y = 0; start_angle = pi;
x_current = [start_x; start_y; start_angle; 0]; 

L = 1.2; Ts = 0.1;
u_prev = [0; 0];
history_x = []; history_u = [];

loops = 0; prev_theta = atan2(start_y, start_x);

% Importante: Parametro RAMP_DIST usato nel training
% Dobbiamo stare APPENA sopra questo valore per non far frenare la rete
RAMP_DIST_TRAINING = 3.0; 

fprintf('Avvio Simulazione NEURALE (NO OVERRIDE)...\n');

for t = 1:1500

    %% DEFINIZIONE TARGET INTELLIGENTE
    theta_polare = atan2(x_current(2), x_current(1));

    if (theta_polare - prev_theta) < -5, loops = loops + 1;
    elseif (theta_polare - prev_theta) > 5, loops = loops - 1; end
    prev_theta = theta_polare;

    current_arc = theta_polare + 2*pi*loops;

    % --- CALCOLO LOOK-AHEAD DINAMICO ---
    % Se vogliamo andare a V_target, dobbiamo proiettare il target
    % a una distanza tale che la rete NON freni.
    
    % Minimo vitale: RAMP_DIST + piccolo margine (es. 0.2m)
    % Se scendiamo sotto 3.0m, la rete inchioda.
    min_look_ahead = RAMP_DIST_TRAINING + 0.2; 
    
    % Look-ahead basato sulla velocità (tipo Pure Pursuit)
    % K_gain determina quanto "anticipiamo" la curva
    K_gain = 2.0; 
    look_ahead_dist = max(min_look_ahead, V_target * K_gain);
    
    % Saturiamo per non guardare TROPPO avanti (altrimenti taglia la curva)
    look_ahead_dist = min(look_ahead_dist, 3.5);

    angle_target = current_arc + (look_ahead_dist / R_circle);

    xt = R_circle * cos(angle_target);
    yt = R_circle * sin(angle_target);
    th_t = angle_target + pi/2; 

   %% TRASFORMAZIONE COORDINATE
    x_rel = x_current(1) - xt; 
    y_rel = x_current(2) - yt; 
    th_rel = atan2(sin(x_current(3)), cos(x_current(3))); 
    
    state_for_net = [x_rel; y_rel; th_rel; x_current(4)];

    %% RETE NEURALE (NESSUN TRUCCO)
    input_vec = [state_for_net; u_prev];
    input_norm = (input_vec' - mu_X) ./ sig_X;

    % Predizione Pura
    u_opt = predict(net, input_norm)';

    % Qui la rete decide TUTTO. 
    % Se il look_ahead_dist è > 3.0m, la rete DOVREBBE dare gas.
    
    % Saturazione di sicurezza (solo limiti fisici)
    u_opt(1) = max(min(u_opt(1), 3.0), -1.0); 
    u_opt(2) = max(min(u_opt(2), 0.3), -0.3); % Sterzo max aumentato un filo a 0.3

    %% FISICA
    dxdt = Car_Like_Model(x_current, u_opt, L);
    x_current = x_current + dxdt * Ts;
    u_prev = u_opt;
    history_x = [history_x; x_current'];
    history_u = [history_u; u_opt'];
end

%% VISUALIZZAZIONE
figure(1); clf; set(gcf,'Color','w');
hold on; axis equal; grid on;
plot(R_circle*cos(0:0.01:2*pi), R_circle*sin(0:0.01:2*pi), 'k--');
plot(history_x(:,1), history_x(:,2), 'b-', 'LineWidth', 2);
title(['Guida Neurale Pura - LookAhead Dinamico']);

figure(2); clf; set(gcf,'Color','w');
subplot(2,1,1); plot(history_u(:,1)); title('Velocità Decisa dalla Rete'); grid on; yline(V_target,'r--');
subplot(2,1,2); plot(history_u(:,2)); title('Sterzo Deciso dalla Rete'); grid on;