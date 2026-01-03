%% RACCOLTA DATI STANDARD (Traiettorie Complete & Fluide)
% Genera un dataset di episodi completi (Start -> Target).
% Basato fedelmente su: Simulazione_singola_traiettoria_Non_Lineare.m

clear all; clc; close all;

%% PARAMETRI DEL SISTEMA (Identici al tuo script)
L = 1.2;         
Ts = 0.1;        
N = 40;          

TOTAL_SAMPLES = 15000; 
OUTPUT_FILE = 'Dataset_Standard_Coherent.mat';

% VINCOLI FISICI (Statici per generazione, poi sovrascritti dal Planner)
u_min = [-2.0; -0.3]; % Sterzo limitato a 0.3 come nel tuo script
u_max = [ 2.0;  0.3]; 

% VINCOLI DINAMICI (Slew Rate)
du_min = [-0.5; -0.1]; 
du_max = [ 0.5;  0.1]; 

% MATRICI PESO (Le tue)
C = eye(4);
Q = diag([30, 30, 1, 0]); % [x, y, theta, phi]
R = diag([10, 50]);       % [v, w]

% SOGLIE PLANNER
RAMP_DIST     = 3.0;  
STOP_DIST     = 0.30; 
TOLERANCE_POS = 0.20; 

%% INIZIALIZZAZIONE
X_dataset = zeros(TOTAL_SAMPLES, 6); 
Y_dataset = zeros(TOTAL_SAMPLES, 2);

fprintf('Avvio Raccolta Dati STANDARD (Traiettorie)...\n');

counter = 0;
episode_count = 0;

%% LOOP GENERAZIONE
while counter < TOTAL_SAMPLES
    episode_count = episode_count + 1;
    
    % --- A. CONFIGURAZIONE EPISODIO ---
    y_ref_target = [0; 0];
    
    % Partenza casuale (10-25m)
    dist_start = 10 + 15 * rand();
    angle_start = -pi + 2*pi * rand();
    
    start_x = dist_start * cos(angle_start);
    start_y = dist_start * sin(angle_start);
    
    % Orientamento "buono" (verso il target) con un po' di rumore
    perfect_angle = atan2(-start_y, -start_x);
    start_theta = perfect_angle + (rand()-0.5) * (pi/2);
    
    start_phi = 0; 
    u_prev = [0; 0]; 
    
    x_current = [start_x; start_y; start_theta; start_phi];
    final_approach_angle = 0; % Memoria angolo
    
    % SIMULAZIONE (Max 400 passi) ---
    for t = 1:400
        if counter >= TOTAL_SAMPLES, break; end
        
        % CHECK DISTANZA
        dist = norm(x_current(1:2) - y_ref_target);
        
        if dist < TOLERANCE_POS && norm(u_prev) < 0.1
            break; % Successo
        end
        
        % LOGICA PLANNER (Copiata dal tuo script)
        dx_t = y_ref_target(1) - x_current(1);
        dy_t = y_ref_target(2) - x_current(2);
        
        % Pure Pursuit
        if dist > 1.0
            desired_theta = atan2(dy_t, dx_t);
            final_approach_angle = desired_theta;
        else
            desired_theta = final_approach_angle;
        end
        
        % Soft Landing
        if dist > RAMP_DIST
            v_target = 2.0;
        else
            v_target = 0.3 + (2.0 - 0.3) * (dist / RAMP_DIST);
        end
        
        % Aggiornamento Vincoli Dinamici (Sterzo a 0.3)
        curr_u_max = [ v_target;  0.3];
        curr_u_min = [-v_target; -0.3];
        
        % Angle Wrapping
        delta_theta = x_current(3) - desired_theta;
        delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));
        
        x_mpc_input = x_current;
        x_mpc_input(3) = desired_theta + delta_theta_norm;
        
        % Riferimento
        current_y_ref = [y_ref_target(1); y_ref_target(2); desired_theta; 0];
        Y_ref_Long = reshape(repmat(current_y_ref, 1, N), [], 1);
        
        % 3. MPC (ESPERTO)
        try
            [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev, Y_ref_Long, N, Ts, L, Q, R, ...
                curr_u_min, curr_u_max, du_min, du_max, C);
        catch
            u_opt = [0;0];
        end
        
        % 4. SALVATAGGIO
        counter = counter + 1;
        X_dataset(counter, :) = [x_current', u_prev'];
        Y_dataset(counter, :) = u_opt';
        
        % 5. FISICA REALE (Non Lineare)
        dxdt = Car_Like_Model(x_current, u_opt, L);
        x_next = x_current + dxdt * Ts;
        
        x_current = x_next;
        u_prev = u_opt;
    end
    
    if mod(counter, 2000) < 50
        fprintf('Generati %d/%d campioni...\n', counter, TOTAL_SAMPLES);
    end
end

save(OUTPUT_FILE, 'X_dataset', 'Y_dataset', 'L', 'Ts', 'N');
fprintf('Dataset STANDARD salvato.\n');