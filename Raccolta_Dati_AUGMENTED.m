%% RACCOLTA DATI MPC - AUGMENTED (Data Augmentation) - STANDARDIZED
% FIX: Rimossi valori hardcoded (2.0, 0.3, etc).
% FIX: Coerenza totale limiti V e W.

clear all; clc; close all;

%% 1. PARAMETRI DEL SISTEMA
L = 1.2;         
Ts = 0.1;        
N = 40;          

TARGET_PHYSICAL_STEPS = 8000; 
N_aug = 7; 

OUTPUT_FILE = 'Dataset_Augmented_Final.mat';

%% --- PARAMETRI FISICI E LIMITI (GLOBAL CONFIG) ---
% Definiscili QUI una volta per tutte
V_MAX_SYS = 3.0;   % Limite fisico assoluto velocità [m/s]
W_MAX_SYS = 0.35;  % Limite fisico assoluto sterzo [rad] (~20 gradi)

V_CRUISE  = 2.0;   % Velocità target di crociera (quella che vuole il planner)
V_REVERSE = -0.5;  % Velocità massima in retromarcia

% Limiti Accelerazione
ACC_MAX = 1.0;
ACC_MIN = -1.0; 
D_STEER_MAX = 0.2; % Variazione sterzo max per passo

%% PARAMETRI MPC
% Vincoli delta u (Accelerazione)
du_min = [ACC_MIN; -D_STEER_MAX];
du_max = [ACC_MAX;  D_STEER_MAX];

C = eye(4);
Q = diag([2, 2, 50, 0]);   
R = diag([1, 150]);        

% Parametri Planner
RAMP_DIST     = 1.5;  
TOLERANCE_POS = 0.20; 

% Limiti per Data Augmentation (basati sui fisici)
w_aug_min = -0.2;       % Range safe per augmentation
w_aug_max =  0.2;

%% INIZIALIZZAZIONE
ESTIMATED_ROWS = TARGET_PHYSICAL_STEPS * (N_aug + 1);
X_dataset = zeros(ESTIMATED_ROWS, 6); 
Y_dataset = zeros(ESTIMATED_ROWS, 2);

fprintf('Avvio Raccolta AUGMENTED (V_MAX=%.1f, W_MAX=%.2f)...\n', V_MAX_SYS, W_MAX_SYS);

counter_rows = 0;   
counter_steps = 0;  

%% LOOP GENERAZIONE
while counter_steps < TARGET_PHYSICAL_STEPS
    
    % --- START RANDOM ---
    y_ref_target = [0; 0]; 
    dist_start = 10 + 15 * rand(); 
    angle_start = -pi + 2*pi * rand();
    
    start_x = dist_start * cos(angle_start);
    start_y = dist_start * sin(angle_start);
    
    perfect_angle = atan2(-start_y, -start_x);
    start_theta = perfect_angle + (rand()-0.5) * (pi/2);
    
    x_current = [start_x; start_y; start_theta; 0]; 
    u_prev_real = [0; 0]; 
    final_approach_angle = 0;
    
    for t = 1:400
        if counter_steps >= TARGET_PHYSICAL_STEPS, break; end
        
        dist = norm(x_current(1:2) - y_ref_target);
        dx_t = y_ref_target(1) - x_current(1);
        dy_t = y_ref_target(2) - x_current(2);
        
        if dist > 1.0
            desired_theta = atan2(dy_t, dx_t);
            final_approach_angle = desired_theta;
        else
            desired_theta = final_approach_angle;
        end
        
        % --- PLANNER VELOCITÀ (NO HARDCODING) ---
        if dist > RAMP_DIST
            v_target_curr = V_CRUISE; % Usa la variabile, non 2.0
        else
            v_target_curr = 0.3 + (V_CRUISE - 0.3) * (dist / RAMP_DIST);
        end
        
        % Vincoli Dinamici Locali per MPC
        % Qui usiamo W_MAX_SYS invece di 0.3 fisso
        curr_u_max = [ v_target_curr;  W_MAX_SYS];
        curr_u_min = [-v_target_curr; -W_MAX_SYS];
        
        % Input MPC
        delta_theta = x_current(3) - desired_theta;
        delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));
        x_mpc_input = x_current;
        x_mpc_input(3) = desired_theta + delta_theta_norm;
        
        % Orizzonte Passante
        Ref_Horizon = zeros(4, N);
        for k = 1:N
            dist_k = (k-1) * v_target_curr * Ts; 
            Ref_Horizon(1,k) = y_ref_target(1) + dist_k * cos(desired_theta);
            Ref_Horizon(2,k) = y_ref_target(2) + dist_k * sin(desired_theta);
            Ref_Horizon(3,k) = desired_theta; 
            Ref_Horizon(4,k) = 0; 
        end
        
        % Unwrapping
        base_angle = x_mpc_input(3);
        d_theta = Ref_Horizon(3,1) - base_angle;
        d_theta = atan2(sin(d_theta), cos(d_theta));
        Ref_Horizon(3,1) = base_angle + d_theta;
        for k = 2:N
            diff = Ref_Horizon(3,k) - Ref_Horizon(3,k-1);
            diff = atan2(sin(diff), cos(diff));
            Ref_Horizon(3,k) = Ref_Horizon(3,k-1) + diff;
        end
        Y_ref_Long = reshape(Ref_Horizon, [], 1);
        
        % --- DATA AUGMENTATION (COERENTE) ---
        u_prev_candidates = zeros(N_aug + 1, 2);
        u_prev_candidates(1, :) = u_prev_real'; 
        
        overshoot_margin = 0.5; 
        
        % Calcolo limiti casuali coerenti con i limiti di sistema
        local_v_max_aug = min(v_target_curr + overshoot_margin, V_MAX_SYS);
        local_v_min_aug = V_REVERSE;
        
        rand_v = local_v_min_aug + (local_v_max_aug - local_v_min_aug) * rand(N_aug, 1);
        rand_w = w_aug_min + (w_aug_max - w_aug_min) * rand(N_aug, 1);
        u_prev_candidates(2:end, :) = [rand_v, rand_w];
        
        u_opt_real_step = [0;0];
        
        for k = 1:size(u_prev_candidates, 1)
            u_prev_test = u_prev_candidates(k, :)';
            try
                [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev_test, Y_ref_Long, N, Ts, L, Q, R, ...
                    curr_u_min, curr_u_max, du_min, du_max, C);
            catch
                u_opt = [0;0]; 
            end
            
            counter_rows = counter_rows + 1;
            X_dataset(counter_rows, :) = [x_current', u_prev_test'];
            Y_dataset(counter_rows, :) = u_opt';
            
            if k == 1, u_opt_real_step = u_opt; end
        end
        
        dxdt = Car_Like_Model(x_current, u_opt_real_step, L);
        x_next = x_current + dxdt * Ts;
        x_current = x_next;
        u_prev_real = u_opt_real_step;
        counter_steps = counter_steps + 1;
        
        if dist < TOLERANCE_POS && norm(u_prev_real) < 0.1, break; end
    end
    
    if mod(counter_steps, 1000) < 50
        fprintf('Passi Fisici: %d / %d ...\n', counter_steps, TARGET_PHYSICAL_STEPS);
    end
end

X_dataset = X_dataset(1:counter_rows, :);
Y_dataset = Y_dataset(1:counter_rows, :);
save(OUTPUT_FILE, 'X_dataset', 'Y_dataset', 'L', 'Ts', 'N');
fprintf('DATASET COMPLETATO.\n');