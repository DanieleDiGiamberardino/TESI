%% RACCOLTA DATI RECOVERY (Punti Singoli) - STANDARDIZED
% FIX: Rimossa incoerenza limiti (es. 0.3 vs 0.35).

clear all; clc; close all;

%% PARAMETRI
L = 1.2; Ts = 0.1; N = 40;

TARGET_SCENARIOS = 10000; 
N_aug = 5;                
OUTPUT_FILE = 'Dataset_Recovery_Augmented.mat';

%% --- PARAMETRI FISICI (DEVONO MATCHARE L'ALTRO SCRIPT) ---
V_MAX_SYS = 3.0;   
W_MAX_SYS = 0.35;  

V_CRUISE  = 2.0;   
V_REVERSE = -0.5;

ACC_MAX = 1.0;
ACC_MIN = -1.0; 
D_STEER_MAX = 0.2;

% Limiti Assoluti per Data Augmentation
u_min = [V_REVERSE; -W_MAX_SYS]; 
u_max = [V_MAX_SYS;  W_MAX_SYS]; 

du_min = [ACC_MIN; -D_STEER_MAX]; 
du_max = [ACC_MAX;  D_STEER_MAX];

C = eye(4);
Q = diag([5, 5, 50, 0]);   
R = diag([1, 100]); 
RAMP_DIST = 1.5; 

%% INIZIALIZZAZIONE
ESTIMATED_ROWS = TARGET_SCENARIOS * N_aug;
X_dataset = zeros(ESTIMATED_ROWS, 6);
Y_dataset = zeros(ESTIMATED_ROWS, 2);

fprintf('Avvio Raccolta RECOVERY (Standardized)...\n');

counter_rows = 0;
counter_scenarios = 0;

while counter_scenarios < TARGET_SCENARIOS
    
    % Generazione casuale (uguale a prima)
    dist_rand = 0.5 + 19.5 * rand();
    angle_to_target_rel = -pi + 2*pi * rand(); 
    x_val = -dist_rand * cos(angle_to_target_rel);
    y_val = -dist_rand * sin(angle_to_target_rel);
    theta_val = -pi + 2*pi * rand(); 
    phi_val = -W_MAX_SYS + (2*W_MAX_SYS) * rand(); % Sterzo random nei limiti corretti
    
    x_current = [x_val; y_val; theta_val; phi_val];

    dist = norm([x_val; y_val]);
    dx_t = -x_val; dy_t = -y_val;
    desired_theta = atan2(dy_t, dx_t);
    
    heading_err = desired_theta - theta_val;
    heading_err = atan2(sin(heading_err), cos(heading_err));
    
    % Logica Velocità Coerente
    if abs(heading_err) > pi/2
        v_target_planner = 0.5;
    elseif dist > RAMP_DIST
        v_target_planner = V_CRUISE; % Usa variabile
    else
        v_target_planner = 0.3 + (V_CRUISE - 0.3)*(dist/RAMP_DIST);
    end

    delta_theta = theta_val - desired_theta;
    delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));
    x_mpc_input = x_current;
    x_mpc_input(3) = desired_theta + delta_theta_norm;

    current_y_ref = [0; 0; desired_theta; 0];
    Y_ref_Long = reshape(repmat(current_y_ref, 1, N), [], 1);
    
    for k = 1:N_aug
        
        v_rand_min = V_REVERSE;
        % Max possibile per augmentation (overshoot permesso ma saturato a V_MAX_SYS)
        v_rand_max = min(v_target_planner + 0.5, V_MAX_SYS);
        
        rand_v = v_rand_min + (v_rand_max - v_rand_min) * rand();
        rand_w = -W_MAX_SYS + (2*W_MAX_SYS) * rand(); % Random sterzo completo
        u_prev_test = [rand_v; rand_w];
        
        curr_u_max = [v_target_planner;  W_MAX_SYS];
        curr_u_min = [-v_target_planner; -W_MAX_SYS];
        
        try
            [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev_test, Y_ref_Long, N, Ts, L, Q, R, ...
                curr_u_min, curr_u_max, du_min, du_max, C);
        catch
            u_opt = [0;0];
        end
        
        counter_rows = counter_rows + 1;
        X_dataset(counter_rows, :) = [x_val, y_val, theta_val, phi_val, u_prev_test(1), u_prev_test(2)];
        Y_dataset(counter_rows, :) = u_opt';
    end
    
    counter_scenarios = counter_scenarios + 1;
    if mod(counter_scenarios, 2000) == 0
        fprintf('Scenari: %d...\n', counter_scenarios);
    end
end

X_dataset = X_dataset(1:counter_rows, :);
Y_dataset = Y_dataset(1:counter_rows, :);
save(OUTPUT_FILE, 'X_dataset', 'Y_dataset', 'L', 'Ts', 'N');
fprintf('DATASET RECOVERY COMPLETATO.\n');