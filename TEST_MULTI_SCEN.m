%% TEST MULTI-SCENARIO: APPROCCIO SEMPLIFICATO "PURE PURSUIT"
clear all; clc; close all;

%% 1. CONFIGURAZIONE GLOBALE
% Parametri Modello
L = 1.2; Ts = 0.1; N = 30;

% Target [x; y] (L'angolo non lo forziamo più a 0 fisso)
y_target_pos = [-10; -5]; 

% --- TUNING SEMPLIFICATO ---
% Q: [Pos_X, Pos_Y, Theta, Phi]
% NOTA: Theta ha peso 1.0 (Basso). L'auto non si stressa per l'angolo.
Q = diag([30, 30, 1.0, 0]); 

% R: [Velocità, Vel_Sterzata]
R = diag([10, 50]);

C = eye(4);

% Vincoli
u_min_abs = [-3.0; -0.3]; 
u_max_abs = [ 5.0;  0.3];
du_min    = [-0.5; -0.1]; 
du_max    = [ 0.5;  0.1];

% Soglie di Stop
STOP_DIST      = 0.30; 
TOLERANCE_POS  = 0.20; % Un po' più larga per evitare jitter finale
RAMP_DIST      = 5.0;  

%% 2. DEFINIZIONE SCENARI
scenarios = [
     15,  20,  180;   % 1. Lontano Nord-Est
    -20,  20, -90;    % 2. Lontano Nord-Ovest
    -25, -15,   0;    % 3. Lontano Sud-Ovest
     10, -20,  90;    % 4. Lontano Sud-Est
     -5,   5, -45;    % 5. Vicino Nord
    -15,  -5,   0;    % 6. Vicino Ovest
    -10, -15,  90;    % 7. Vicino Sud
      0,  -5, 180;    % 8. Vicino Est (Critico)
];

num_scenarios = size(scenarios, 1);
results = []; 

figure('Color','w', 'Name', 'Pure Pursuit Convergence');
hold on; grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
viscircles(y_target_pos', STOP_DIST, 'Color', [0.8 0 0], 'LineStyle', '-');
plot(y_target_pos(1), y_target_pos(2), 'pk', 'MarkerSize', 15, 'MarkerFaceColor', 'r');

fprintf('Avvio Test Semplificato (Pure Pursuit)...\n\n');

%% 3. CICLO DI SIMULAZIONE
for s = 1:num_scenarios
    sx = scenarios(s, 1); sy = scenarios(s, 2); st = deg2rad(scenarios(s, 3));
    x_current = [sx; sy; st; 0];
    u_prev    = [0; 0];
    
    history_x = []; 
    converged = false;
    steps_taken = 0;
    
    % Variabile per congelare l'angolo finale
    final_approach_angle = 0;
    freeze_angle = false;

    fprintf('Scenario %d/%d...', s, num_scenarios);
    
    for t = 1:2000 
        dist = norm(x_current(1:2) - y_target_pos);
        
        % === DEADZONE (STOP) ===
        if dist < TOLERANCE_POS
            u_opt = [0; 0];
            if t > 10 && norm(u_prev) < 0.05 && dist < STOP_DIST
                converged = true; steps_taken = t; break; 
            end
        else
            % =====================================================
            % === LOGICA "PURE PURSUIT" (SEMPLIFICATA)          ===
            % =====================================================
            
            % 1. Calcolo Vettore Direzione
            dx_t = y_target_pos(1) - x_current(1);
            dy_t = y_target_pos(2) - x_current(2);
            
            % 2. Calcolo Angolo Desiderato (Punta sempre al target)
            % Se siamo molto vicini (< 1m), congeliamo l'angolo per non
            % farlo impazzire quando passa sopra il punto esatto.
            if dist > 1.0
                desired_theta = atan2(dy_t, dx_t);
                final_approach_angle = desired_theta; % Memorizza ultimo buono
            else
                desired_theta = final_approach_angle; % Mantieni direzione
            end
            
            % Costruiamo il vettore di riferimento completo
            % Target Posizione + Target Angolo calcolato dinamicamente
            current_y_ref = [y_target_pos(1); y_target_pos(2); desired_theta; 0];
            
            % 3. Speed Profile
            if dist > RAMP_DIST
                v_target = 3.0;
            else
                v_target = 0.3 + (3.0 - 0.3) * (dist / RAMP_DIST);
            end
            
            curr_u_max = [v_target;  0.3];
            curr_u_min = [-v_target; -0.3];
            
            % 4. MPC Prep (Angle Wrapping essenziale)
            delta_th = x_current(3) - desired_theta;
            delta_th_norm = atan2(sin(delta_th), cos(delta_th));
            
            x_mpc = x_current;
            x_mpc(3) = desired_theta + delta_th_norm;
            
            Y_ref_Long = reshape(repmat(current_y_ref, 1, N), [], 1);
            
            try
                [u_opt, ~] = MPC_Linear(x_mpc, u_prev, Y_ref_Long, N, Ts, L, Q, R, ...
                    curr_u_min, curr_u_max, du_min, du_max, C);
            catch
                u_opt = u_prev;
            end
        end
        
        % Fisica
        dxdt = Car_Like_Model(x_current, u_opt, L);
        x_current = x_current + dxdt * Ts;
        u_prev = u_opt;
        history_x = [history_x; x_current'];
        if t == 2000, steps_taken = 2000; end
    end
    
    % Salvataggio
    final_err_pos = norm(x_current(1:2) - y_target_pos);
    results = [results; s, converged, steps_taken, final_err_pos];
    
    % Plotting
    if converged
        plot(history_x(:,1), history_x(:,2), 'b-', 'LineWidth', 1.2);
        plot(sx, sy, 'go', 'MarkerSize', 6, 'LineWidth', 2); 
        fprintf(' OK (Err: %.3fm)\n', final_err_pos);
    else
        plot(history_x(:,1), history_x(:,2), 'r--', 'LineWidth', 1.5);
        plot(sx, sy, 'rx', 'MarkerSize', 8, 'LineWidth', 2); 
        fprintf(' FAIL\n');
    end
    drawnow;
end

%% 4. REPORT
fprintf('\n======================================================\n');
fprintf(' REPORT SEMPLIFICATO (PURE PURSUIT) \n');
fprintf('======================================================\n');
fprintf('| # | Start [X, Y]   | Status     | Steps | Err [m] |\n');
fprintf('|---|----------------|------------|-------|---------|\n');
for k = 1:num_scenarios
    st_str = 'FAILED'; if results(k, 2) == 1, st_str = 'SUCCESS'; end
    fprintf('| %d | [%5.1f, %5.1f] | %-10s | %5d | %7.4f |\n', ...
        results(k,1), scenarios(k,1), scenarios(k,2), st_str, results(k,3), results(k,4));
end