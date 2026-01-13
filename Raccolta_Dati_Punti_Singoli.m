%% RACCOLTA DATI RECOVERY (Punti Singoli Coerenti)
% Genera situazioni difficili (Recovery) usando la logica del tuo script.

clear all; clc; close all;

%% PARAMETRI (Coerenti)
L = 1.2; Ts = 0.1; N = 40;
TOTAL_SAMPLES = 20000;
OUTPUT_FILE = 'Dataset_Recovery_SinglePoints.mat';

% VINCOLI E PESI
u_min = [-2.0; -0.3]; u_max = [ 2.0;  0.3]; % Sterzo max 0.3
du_min = [-0.5; -0.1]; du_max = [ 0.5;  0.1];
C = eye(4); Q = diag([30, 30, 1, 0]); R = diag([10, 50]);
RAMP_DIST = 3.0;

%% INIZIALIZZAZIONE
X_dataset = zeros(TOTAL_SAMPLES, 6);
Y_dataset = zeros(TOTAL_SAMPLES, 2);

fprintf('Avvio Raccolta Dati RECOVERY...\n');

for i = 1:TOTAL_SAMPLES

    %  STATO CASUALE
    dist_rand = 0.5 + 19.5 * rand();
    angle_rand = -pi + 2*pi * rand();

    x_val = dist_rand * cos(angle_rand);
    y_val = dist_rand * sin(angle_rand);

    % Orientamento anche contromano
    theta_val = -pi + 2*pi * rand();
    phi_val = -0.3 + 0.6 * rand(); % Sterzo entro +/- 0.3

    % U_PREV COERENTE (Fisica inversa)
    if dist_rand > RAMP_DIST
        v_base = 2.0;
    else
        v_base = 0.3 + (2.0 - 0.3)*(dist_rand/RAMP_DIST);
    end
    v_prev_gen = v_base + (rand()-0.5)*1.0;

    % Sterzo precedente coerente
    w_prev_gen = (rand()-0.5) * 0.1;

    % Saturazione
    v_prev_gen = max(min(v_prev_gen, u_max(1)), u_min(1));
    w_prev_gen = max(min(w_prev_gen, u_max(2)), u_min(2));

    u_prev_sim = [v_prev_gen; w_prev_gen];
    x_current = [x_val; y_val; theta_val; phi_val];

    % LOGICA PLANNER (La tua logica applicata istantaneamente)
    dist = norm([x_val; y_val]);
    dx_t = -x_val; dy_t = -y_val;

    if dist > 1.0, desired_theta = atan2(dy_t, dx_t);
    else, desired_theta = theta_val; end

    if dist > RAMP_DIST, v_target = 2.0;
    else, v_target = 0.3 + (2.0 - 0.3)*(dist/RAMP_DIST); end

    % Vincoli dinamici corretti
    curr_u_max = [v_target; 0.3];
    curr_u_min = [-v_target; -0.3];

    % Angle Wrapping
    delta_theta = theta_val - desired_theta;
    delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));
    x_mpc_input = x_current;
    x_mpc_input(3) = desired_theta + delta_theta_norm;

    % Riferimento
    current_y_ref = [0; 0; desired_theta; 0];
    Y_ref_Long = reshape(repmat(current_y_ref, 1, N), [], 1);

    % MPC
    try
        [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev_sim, Y_ref_Long, N, Ts, L, Q, R, ...
            curr_u_min, curr_u_max, du_min, du_max, C);
    catch
        u_opt = [0;0];
    end

    X_dataset(i, :) = [x_val, y_val, theta_val, phi_val, u_prev_sim(1), u_prev_sim(2)];
    Y_dataset(i, :) = u_opt';

    if mod(i, 5000) == 0, fprintf('Generati %d/%d...\n', i, TOTAL_SAMPLES); end
end

save(OUTPUT_FILE, 'X_dataset', 'Y_dataset', 'L', 'Ts', 'N');
fprintf('Dataset RECOVERY salvato.\n');