%% RACCOLTA DATI MPC - AUGMENTED (Data Augmentation)
% Questo script genera un dataset arricchito per l'addestramento.
% Per ogni stato fisico visitato, simula diverse condizioni iniziali dei motori (u_prev)
% per insegnare alla rete a gestire qualsiasi situazione pregressa.

clear all; clc; close all;

%% 1. PARAMETRI DEL SISTEMA
L = 1.2;         
Ts = 0.1;        
N = 40;          

% Dimensione totale del dataset desiderata
TOTAL_SAMPLES = 50000; % Aumentato perché ne generiamo tanti per ogni step
OUTPUT_FILE = 'Dataset_Augmented_Final.mat';

%% 2. PARAMETRI AUGMENTATION (Impostazioni del Prof)
% Quante varianti di u_prev generare per ogni singolo stato fisico?
N_aug = 7; 

% LIMITI U_PREV (Guardando i tuoi grafici delle u)
% Velocità (v): oscillava tra circa 0 e 2.0 (o -2 e 2 se consideri tutto)
v_aug_min = -0.5; 
v_aug_max =  3.0;

% Sterzo (w): oscillava tra -0.3 e 0.3
w_aug_min = -0.2;
w_aug_max =  0.2;

%% 3. PARAMETRI CONTROLLORE
% Vincoli sulla u assoluta
u_min = [-0.5; -0.6];
u_max = [ 3;  0.6]; 

% Vincoli sull'accelerazione/decelerazione
du_min = [-1.0; -0.2];
du_max = [ 1.0;  0.2];

% Pesi MPC
C = eye(4);
Q = diag([30, 30, 9, 0]); 
R = diag([7, 50]);       

% Soglie Planner
RAMP_DIST     = 3.0;  
STOP_DIST     = 0.30; 
TOLERANCE_POS = 0.20; 

%% INIZIALIZZAZIONE
% Stimiamo la dimensione per pre-allocare (velocizza)
X_dataset = zeros(TOTAL_SAMPLES, 6); 
Y_dataset = zeros(TOTAL_SAMPLES, 2);

fprintf('Avvio Raccolta Dati AUGMENTED...\n');
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
        
        if dist > RAMP_DIST
            v_target = 2.0;
        else
            v_target = 0.3 + (2.0 - 0.3) * (dist / RAMP_DIST);
        end
        
        curr_u_max = [ v_target;  0.3];
        curr_u_min = [-v_target; -0.3];
        
        delta_theta = x_current(3) - desired_theta;
        delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));
        
        x_mpc_input = x_current;
        x_mpc_input(3) = desired_theta + delta_theta_norm;
        
        current_y_ref = [y_ref_target(1); y_ref_target(2); desired_theta; 0];
        Y_ref_Long = reshape(repmat(current_y_ref, 1, N), [], 1);
        
        % ------------------------------------------------------------
        % SEZIONE DATA AUGMENTATION (Il "trucco" richiesto)
        % ------------------------------------------------------------
        
        % Generiamo una lista di u_prev candidati
        % Include sempre il caso REALE (u_prev_real) + N_aug varianti random
        
        u_prev_candidates = zeros(N_aug + 1, 2);
        
        % 1. Caso Reale (deve esserci sempre per la coerenza fisica)
        u_prev_candidates(1, :) = u_prev_real'; 
        
        % 2. Varianti Casuali (Data Augmentation)
        % Generiamo N_aug valori casuali tra i min e max definiti in alto
        rand_v = v_aug_min + (v_aug_max - v_aug_min) * rand(N_aug, 1);
        rand_w = w_aug_min + (w_aug_max - w_aug_min) * rand(N_aug, 1);
        
        u_prev_candidates(2:end, :) = [rand_v, rand_w];
        
        % Variabile per salvare l'output del caso reale (serve per muovere l'auto)
        u_opt_real_step = [0;0];
        
        % Loop su tutte le varianti (Reale + Immaginarie)
        for k = 1:size(u_prev_candidates, 1)
            
            if counter >= TOTAL_SAMPLES, break; end
            
            u_prev_test = u_prev_candidates(k, :)';
            
            % CHIAMATA MPC
            try
                [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev_test, Y_ref_Long, N, Ts, L, Q, R, ...
                    curr_u_min, curr_u_max, du_min, du_max, C);
            catch
                u_opt = [0;0];
            end
            
            % SALVATAGGIO NEL DATASET
            counter = counter + 1;
            % Input: Stato Fisico + u_prev (che può essere reale o inventato)
            X_dataset(counter, :) = [x_current', u_prev_test'];
            % Output: La decisione dell'MPC per quella specifica combinazione
            Y_dataset(counter, :) = u_opt';
            
            % Se era il caso reale (indice 1), salviamolo per la fisica
            if k == 1
                u_opt_real_step = u_opt;
            end
        end
        
        % ------------------------------------------------------------
        % FINE AUGMENTATION - AVANZAMENTO FISICO
        % ------------------------------------------------------------
        
        % Muoviamo l'auto SOLO usando il risultato del caso reale (k=1)
        dxdt = Car_Like_Model(x_current, u_opt_real_step, L);
        x_next = x_current + dxdt * Ts;
        
        % Aggiornamento stato reale
        x_current = x_next;
        u_prev_real = u_opt_real_step;
        
        % Controllo arrivo al target (per terminare episodio)
        if dist < TOLERANCE_POS && norm(u_prev_real) < 0.1
            break; 
        end
    end
    
    if mod(counter, 5000) < 100
        fprintf('Campioni raccolti: %d / %d...\n', counter, TOTAL_SAMPLES);
    end
end

%% RIDUZIONE E SALVATAGGIO
% Tagliamo le righe vuote se abbiamo sforato di poco il loop o interrotto
X_dataset = X_dataset(1:counter, :);
Y_dataset = Y_dataset(1:counter, :);

fprintf('Salvataggio su file: %s ...\n', OUTPUT_FILE);
save(OUTPUT_FILE, 'X_dataset', 'Y_dataset', 'L', 'Ts', 'N');
fprintf('COMPLETATO! Ora puoi addestrare la rete con questi dati.\n');