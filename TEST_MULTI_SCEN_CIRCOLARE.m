%% TEST MULTI "CERCHIO"
clear all; clc; close all;

%% PARAMETRI FISSI
L = 1.2; Ts = 0.1;
N = 30;

C = eye(4);

% TUNING
Q = diag([15, 15, 5, 0]);

R = diag([5, 50]);

% Vincoli
u_min = [-0.5; -0.6];
u_max = [ 3.0;  0.8];
du_min = [-1.0; -0.2];
du_max = [ 1.0;  0.2];

%% CREO GLI SCENARI
scenarios = [
    10,   20,   -5,  180;   % 1.
    5,    1,    1,   45;   % 2.
    15,  -25,  -20,    0;   % 3.
    8,   15,   10,  -90;   % 4.
    ];

num_scenarios = size(scenarios, 1);
results = [];

figure('Color','w', 'Name', 'Circle Test', 'Position', [100, 100, 1000, 800]);

%%  CICLO SIMULAZIONE
for s = 1:num_scenarios
    % Setup stato corrente e Cerchio
    R_circle = scenarios(s, 1);
    sx = scenarios(s, 2); sy = scenarios(s, 3); st = deg2rad(scenarios(s, 4));

    x_current = [sx; sy; st; 0];
    u_prev = [0; 0];

    % VELOCITÀ
    V_target_base = sqrt(0.4 * R_circle);
    V_target = max(0.8, min(1.8, V_target_base));

    history_x = [];
    errors_steady = [];

    loops = 0;
    prev_theta_car = atan2(sy, sx);

    fprintf('Scenario %d (R=%dm): V=%.2f m/s... ', s, R_circle, V_target);

    steps_tot = 1500;
    for t = 1:steps_tot

        % Come nel caso singolo
        theta_car = atan2(x_current(2), x_current(1));
        if (theta_car - prev_theta_car) < -5, loops = loops + 1;
        elseif (theta_car - prev_theta_car) > 5, loops = loops - 1; end
        prev_theta_car = theta_car;

        current_arc_angle = theta_car + 2*pi*loops;
        step_theta = (V_target / R_circle) * Ts;

        % Costruzione Orizzonte
        Ref_Horizon = zeros(4, N);
        for k = 1:N
            ang_k = current_arc_angle  + (k-1)*step_theta;

            ref_x = R_circle * cos(ang_k);
            ref_y = R_circle * sin(ang_k);
            ref_theta = ang_k + pi/2;

            Ref_Horizon(1,k) = ref_x;
            Ref_Horizon(2,k) = ref_y;
            Ref_Horizon(3,k) = ref_theta;
            Ref_Horizon(4,k) = atan(L/R_circle);
        end


        delta = x_current(3) - Ref_Horizon(3,1);
        delta_norm = atan2(sin(delta), cos(delta));
        x_mpc_input = x_current;
        x_mpc_input(3) = Ref_Horizon(3,1) + delta_norm;

        base_angle = x_mpc_input(3);
        for k = 1:N
            diff = Ref_Horizon(3,k) - base_angle;
            diff = atan2(sin(diff), cos(diff));
            Ref_Horizon(3,k) = base_angle + diff;
            base_angle = Ref_Horizon(3,k);
        end

        y_ref_long = reshape(Ref_Horizon, [4*N, 1]);

        % MPC
        try
            [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev, y_ref_long, N, Ts, L, Q, R, u_min, u_max, du_min, du_max, C);
        catch
            u_opt = u_prev;
        end

        % Fisica
        dxdt = Car_Like_Model(x_current, u_opt, L);
        x_current = x_current + dxdt * Ts;
        u_prev = u_opt;

        history_x = [history_x; x_current'];

        % Errore Regime (Seconda metà sim)
        if t > steps_tot/2
            dist_center = norm(x_current(1:2));
            errors_steady = [errors_steady; abs(dist_center - R_circle)];
        end
    end

    avg_err = mean(errors_steady);
    results = [results; s, R_circle, avg_err];
    fprintf('Avg Err: %.3fm\n', avg_err);

    % Plot
    subplot(2, 2, s);
    hold on; grid on; axis equal;
    th = 0:0.01:2*pi;
    plot(R_circle*cos(th), R_circle*sin(th), 'k--', 'LineWidth', 1); % Riferimento
    plot(history_x(:,1), history_x(:,2), 'b-', 'LineWidth', 1.5);    % Percorso
    plot(sx, sy, 'go', 'MarkerSize', 6, 'LineWidth', 2);
    title(sprintf('Scenario %d (R=%dm)\nErr: %.3fm', s, R_circle, avg_err));
end