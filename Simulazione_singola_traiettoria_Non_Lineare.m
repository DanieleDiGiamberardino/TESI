%% SIMULAZIONE SINGOLA:
% Vai al punto X,Y ignorando l'orientamento finale
clear all; clc; close all;

%% CONFIGURAZIONE SCENARIO
start_x = 15;
start_y = 20;
% Calcoliamo un angolo di partenza
start_angle = atan2(-start_y, -start_x) + deg2rad(0);

% Target [x; y; theta; phi]
y_ref = [-10; -5; 0; 0];

%% PARAMETRI E TUNING
L = 1.2; Ts = 0.1; N = 40;

% SOGLIE
STOP_DIST      = 0.30; % Raggio di stop fisico
TOLERANCE_POS  = 0.20; % Tolleranza Deadzone
RAMP_DIST      = 3.0;  % Distanza inizio frenata dolce

C = eye(4);
% Q = [Pos_X, Pos_Y, Theta, Phi]
Q = diag([30, 30, 1, 0]);

% R: [Velocità, Vel_Sterzata]
R = diag([10, 50]);

% Vincoli sull'accelerazione/decelerazione
du_min    = [-0.5; -0.1];
du_max    = [ 0.5;  0.1];

%% INIZIALIZZAZIONE
x_current = [start_x; start_y; start_angle; 0];
u_prev    = [0; 0];

history_x = [x_current'];
history_u = [];

% Variabili per gestione angolo finale
final_approach_angle = 0;
freeze_angle = false;

fprintf('Avvio Simulazione\n');

%% LOOP DI SIMULAZIONE
for t = 1:2500
    % Distanza Euclidea dal Target
    dist = norm(x_current(1:2) - y_ref(1:2));

    %% DEADZONE
    if dist < TOLERANCE_POS
        % Siamo arrivati: Spegni i motori
        u_opt = [0; 0];

        % Check se siamo fermi da un po' e sono entro una distanza STOP_DIST per uscire dal loop(cosi mi salvo
        % gli ultimi stati)
        if t > 10 && norm(u_prev) < 0.05 && dist < STOP_DIST
            fprintf('TARGET RAGGIUNTO (Step %d)\n', t);
            break;
        end

    else
        % PLANNER
        % CALCOLO ANGOLO
        dx_t = y_ref(1) - x_current(1);
        dy_t = y_ref(2) - x_current(2);

        %% Calcolo Angolo di Puntamento
        % Se siamo lontani (>1m), aggiorno l'angolo verso il target
        % Se siamo vicini (<1m), CONGELIAMO l'ultimo angolo buono
        if dist > 1.0
            desired_theta = atan2(dy_t, dx_t); % calcoliamo l angolo desiderato per puntare verso il target
            final_approach_angle = desired_theta; % Memorizza ultimo angolo
        else
            desired_theta = final_approach_angle; % Mantieni fisso
        end

        % Costruzione Riferimento Dinamico
        current_y_ref = [y_ref(1); y_ref(2); desired_theta; 0];

        % Soft Landing
        if dist > RAMP_DIST
            v_target = 2.0;
        else
            % Rampa lineare da 2.0 a 0.3 m/s
            v_target = 0.2 + (2.0 - 0.2) * (dist / RAMP_DIST);
        end

        % Vincoli dinamici di velocità (Valore assoluto u)
        curr_u_max = [ v_target;  0.2];
        curr_u_min = [-v_target; -0.2];

        %% Angle Wrapping
        % Calcola l'errore minimo (-pi, +pi) e lo aggiungo allo stato attuale
        % per ingannare l'MPC e non fargli fare giri di 360 gradi.
        delta_theta = x_current(3) - desired_theta;
        delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));

        x_mpc_input = x_current;
        x_mpc_input(3) = desired_theta + delta_theta_norm;

        % Costruzione Vettore Orizzonte N
        Y_ref_Long = reshape(repmat(current_y_ref, 1, N), [], 1);

        % CHIAMATA MPC
        try
            [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev, Y_ref_Long, N, Ts, L, Q, R, ...
                curr_u_min, curr_u_max, du_min, du_max, C);
        catch
            u_opt = u_prev;
            fprintf('Warning: MPC fallito allo step %d\n', t);
        end
    end

    % AGGIORNAMENTO FISICO (Modello discreto non lineare)
    dxdt = Car_Like_Model(x_current, u_opt, L);
    x_next = x_current + dxdt * Ts;

    history_x = [history_x; x_next'];
    history_u = [history_u; u_opt'];

    x_current = x_next;
    u_prev    = u_opt;
end

%% ANALISI
% Calcolo indice di arrivo
dist_history = sqrt((history_x(:,1) - y_ref(1)).^2 + (history_x(:,2) - y_ref(2)).^2);
idx_end = length(dist_history);

final_pos = history_x(end, 1:2);
final_err = norm(final_pos - y_ref(1:2)');

fprintf('\n');
fprintf('   REPORT FINALE                               \n');
fprintf('| Distanza Finale dal Target: %8.4f m                      |\n', final_err);
fprintf('| Step Totali:                %8d                       |\n', idx_end);
fprintf('| Angolo Finale (Reale):      %8.4f rad (%6.1f deg)      |\n', history_x(end,3), rad2deg(history_x(end,3)));

%% GRAFICA
figure('Color','w', 'Name', 'Simulazione Singola Pure Pursuit');
hold on; grid on; axis equal;

% Disegna Target e Zone
viscircles(y_ref(1:2)', STOP_DIST, 'Color', [0.8 0 0], 'LineStyle', '--', 'LineWidth', 1.5);
plot(y_ref(1), y_ref(2), 'kp', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); % Stella Rossa Target

% Disegna Traiettoria
plot(history_x(:, 1), history_x(:, 2), 'b-', 'LineWidth', 2);

% Disegna Auto (Start e End)
% Funzione helper per disegnare rettangolo orientato
draw_car = @(x,y,th,col) fill( ...
    x + [cos(th)*-L/2 - sin(th)*-L/4, cos(th)*L/2 - sin(th)*-L/4, cos(th)*L/2 - sin(th)*L/4, cos(th)*-L/2 - sin(th)*L/4], ...
    y + [sin(th)*-L/2 + cos(th)*-L/4, sin(th)*L/2 + cos(th)*-L/4, sin(th)*L/2 + cos(th)*L/4, sin(th)*-L/2 + cos(th)*L/4], ...
    col, 'EdgeColor', 'k', 'FaceAlpha', 0.7);

% Disegna Auto Start (Verde)
draw_car(start_x, start_y, start_angle, [0 1 0]);
text(start_x, start_y+1, 'START', 'FontWeight', 'bold');

% Disegna una freccia Magenta che indica la direzione del "Muso"
arrow_len = 3.0; % Lunghezza della freccia
quiver(start_x, start_y, ...
    cos(start_angle), sin(start_angle), ... % Componenti X e Y del vettore direzione
    arrow_len, ...
    'Color', 'm', ...        % Colore Magenta per vederla bene
    'LineWidth', 2, ...      % Spessore
    'MaxHeadSize', 0.5);     % Dimensione della punta

text(start_x, start_y + 2, 'FRONT', 'Color', 'm', 'FontWeight', 'bold');

% Disegna Auto End (Blu)
draw_car(history_x(end,1), history_x(end,2), history_x(end,3), [0.4 0.7 1.0]);
text(history_x(end,1), history_x(end,2)+1, 'END', 'FontWeight', 'bold', 'Color', 'b');

title(['Traiettoria Target Statico | Err: ' num2str(final_err, '%.3f') 'm']);
xlabel('X [m]'); ylabel('Y [m]');