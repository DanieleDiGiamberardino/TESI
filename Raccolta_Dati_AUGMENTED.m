%% RACCOLTA DATI MPC - AUGMENTED (Data Augmentation) - FIXED
% Questo script genera un dataset arricchito per l'addestramento.
% INCLUDE FIX: Generazione dinamica di u_prev basata su v_target per garantire fattibilità.

clear all; clc; close all;

%% 1. PARAMETRI DEL SISTEMA
L = 1.2;         
Ts = 0.1;        
N = 40;          

% Dimensione totale del dataset desiderata
TOTAL_SAMPLES = 50000; 
OUTPUT_FILE = 'Dataset_Augmented_Final.mat';

%% 2. PARAMETRI AUGMENTATION
% Quante varianti di u_prev generare per ogni singolo stato fisico?
N_aug = 7; 

% Limiti Assoluti per lo sterzo (w)
w_aug_min = -0.2;
w_aug_max =  0.2;

%% PARAMETRI CONTROLLORE
% Vincoli sull'accelerazione/decelerazione
du_min = [-1.0; -0.2];
du_max = [ 1.0;  0.2];

% Pesi MPC
C = eye(4);
Q = diag([5, 5, 50, 0]);   % Theta (50) pesa 10 volte la Posizione (5)
R = diag([1, 100]);        % Sterzare (100) costa tantissimo -> Guida dolce      

% Soglie Planner
RAMP_DIST     = 3.0;  
STOP_DIST     = 0.30; 
TOLERANCE_POS = 0.20; 

%% INIZIALIZZAZIONE
X_dataset = zeros(TOTAL_SAMPLES, 6); 
Y_dataset = zeros(TOTAL_SAMPLES, 2);

fprintf('Avvio Raccolta Dati AUGMENTED (con FIX Dinamico)...\n');
fprintf('Generazione di %d varianti per ogni passo fisico.\n', N_aug);

counter = 0;
episode_count = 0;

%% LOOP GENERAZIONE EPISODI
while counter < TOTAL_SAMPLES
    episode_count = episode_count + 1;
    
    % --- CONFIGURAZIONE EPISODIO (Start Random) ---
    y_ref_target = [0; 0];
    
    dist_start = 10 + 15 * rand(); % Distanza casuale tra 10m e 25m
    angle_start = -pi + 2*pi * rand();
    
    start_x = dist_start * cos(angle_start);
    start_y = dist_start * sin(angle_start);
    
    % Orientamento verso il target +/- rumore
    perfect_angle = atan2(-start_y, -start_x);
    start_theta = perfect_angle + (rand()-0.5) * (pi/2);
    
    x_current = [start_x; start_y; start_theta; 0]; % [x, y, theta, phi]
    
    % Memoria REALE della simulazione
    u_prev_real = [0; 0]; 
    final_approach_angle = 0;
    
    % --- SIMULAZIONE FISICA (Max 400 passi per episodio) ---
    for t = 1:400
        if counter >= TOTAL_SAMPLES, break; end
        
        % 1. CALCOLO RIFERIMENTI (Uguale per tutte le varianti augmentation)
        dist = norm(x_current(1:2) - y_ref_target);
        
        % Planner Logic (Pure Pursuit + Soft Landing)
        dx_t = y_ref_target(1) - x_current(1);
        dy_t = y_ref_target(2) - x_current(2);
        
        if dist > 1.0
            desired_theta = atan2(dy_t, dx_t);
            final_approach_angle = desired_theta;
        else
            desired_theta = final_approach_angle;
        end
        
        % Calcolo velocità target in base alla distanza (Rampa)
        if dist > RAMP_DIST
            v_target = 2.0;
        else
            v_target = 0.3 + (2.0 - 0.3) * (dist / RAMP_DIST);
        end
        
        % Vincoli dinamici per l'MPC attuale
        curr_u_max = [ v_target;  0.3];
        curr_u_min = [-v_target; -0.3];
        
        % Preparazione input MPC
        delta_theta = x_current(3) - desired_theta;
        delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));
        
        x_mpc_input = x_current;
        x_mpc_input(3) = desired_theta + delta_theta_norm;
        
        current_y_ref = [y_ref_target(1); y_ref_target(2); desired_theta; 0];
        Y_ref_Long = reshape(repmat(current_y_ref, 1, N), [], 1);
        
        % ------------------------------------------------------------
        % SEZIONE DATA AUGMENTATION (CORRETTA E DINAMICA)
        % ------------------------------------------------------------
        
        u_prev_candidates = zeros(N_aug + 1, 2);
        
        % 1. Caso Reale (deve esserci sempre per la coerenza fisica)
        u_prev_candidates(1, :) = u_prev_real'; 
        
        % 2. Varianti Casuali (Data Augmentation DINAMICA)
        % Calcoliamo un limite massimo per u_prev che sia fisicamente 
        % compatibile con la frenata richiesta.
        % Se v_target è basso, non possiamo generare una u_prev altissima,
        % altrimenti l'MPC fallisce (Infeasible).
        
        overshoot_margin = 0.9; % Permettiamo di essere un po' più veloci del target (per imparare a frenare)
        local_v_max = v_target + overshoot_margin;
        
        % Saturiamo comunque ai limiti globali del motore (es. 3.0 m/s)
        local_v_max = min(local_v_max, 3.0);
        local_v_min = -0.5; % Un po' di retromarcia è sempre ok
        
        % Generazione casuale nel range FATTIBILE
        rand_v = local_v_min + (local_v_max - local_v_min) * rand(N_aug, 1);
        
        % Per lo sterzo (w) usiamo il range standard
        rand_w = w_aug_min + (w_aug_max - w_aug_min) * rand(N_aug, 1);
        
        u_prev_candidates(2:end, :) = [rand_v, rand_w];
        
        % Variabile per salvare l'output del caso reale
        u_opt_real_step = [0;0];
        
        % Loop su tutte le varianti
        for k = 1:size(u_prev_candidates, 1)
            
            if counter >= TOTAL_SAMPLES, break; end
            
            u_prev_test = u_prev_candidates(k, :)';
            
            % CHIAMATA MPC
            try
                [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev_test, Y_ref_Long, N, Ts, L, Q, R, ...
                    curr_u_min, curr_u_max, du_min, du_max, C);
            catch
                % Se fallisce ancora (raro ora), mettiamo un valore sicuro
                u_opt = [0;0]; 
            end
            
            % SALVATAGGIO NEL DATASET
            counter = counter + 1;
            X_dataset(counter, :) = [x_current', u_prev_test'];
            Y_dataset(counter, :) = u_opt';
            
            % Se era il caso reale (indice 1), salviamolo per la fisica
            if k == 1
                u_opt_real_step = u_opt;
            end
        end
        
        % ------------------------------------------------------------
        % FINE AUGMENTATION - AVANZAMENTO FISICO
        % ------------------------------------------------------------
        
        dxdt = Car_Like_Model(x_current, u_opt_real_step, L);
        x_next = x_current + dxdt * Ts;
        
        x_current = x_next;
        u_prev_real = u_opt_real_step;
        
        if dist < TOLERANCE_POS && norm(u_prev_real) < 0.1
            break; 
        end
    end
    
    if mod(counter, 5000) < 100
        fprintf('Campioni raccolti: %d / %d...\n', counter, TOTAL_SAMPLES);
    end
end

%% RIDUZIONE E SALVATAGGIO
X_dataset = X_dataset(1:counter, :);
Y_dataset = Y_dataset(1:counter, :);

fprintf('Salvataggio su file: %s ...\n', OUTPUT_FILE);
save(OUTPUT_FILE, 'X_dataset', 'Y_dataset', 'L', 'Ts', 'N');
fprintf('COMPLETATO! Ora addestra la rete con questi nuovi dati puliti.\n');