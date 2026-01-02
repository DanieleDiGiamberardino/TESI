%% SIMULAZIONE CERCHIO: FIX DEFINITIVO (Dynamic Horizon)
clear all; clc; close all;

%% CONFIGURAZIONE
R_circle = 10;
V_target = 1.5;   % Velocità sostenuta

% Partenza difficile (Lontana e contromano)
start_x = 20; start_y = -5; start_angle = pi; 
x_current = [start_x; start_y; start_angle; 0];

%% PARAMETRI
L = 1.2; Ts = 0.1; N = 30; 
C = eye(4);

Q = diag([15, 15, 5, 0]); 
% R: Penalizziamo lo sterzo 
R = diag([5, 50]);       

% Vincoli
u_min = [-0.5; -0.6];
u_max = [ 3.0;  0.6]; 
du_min = [-1.0; -0.2];
du_max = [ 1.0;  0.2];

%% SIMULAZIONE
u_prev = [0; 0];
history_x = [];

fprintf('Avvio Simulazione con Orizzonte Dinamico\n');

% Variabili per gestire i giri
loops = 0;
prev_theta_car = atan2(start_y, start_x);

for t = 1:1500 
    theta_car = atan2(x_current(2), x_current(1));
    
    % Gestione Wrapping Giri
    if (theta_car - prev_theta_car) < -5, loops = loops + 1;
    elseif (theta_car - prev_theta_car) > 5, loops = loops - 1; end
    prev_theta_car = theta_car;
    
    % Angolo assoluto 
    current_arc_angle = theta_car + 2*pi*loops;
    
    %  COSTRUZIONE ORIZZONTE DINAMICO 
    % Invece di un punto fisso, calcoliamo N punti futuri sul cerchio.
    Ref_Horizon = zeros(4, N);
    
    % Distanza di anticipo iniziale (Look Ahead)
    look_ahead_angle = 2.0 / R_circle; % Guardiamo 2 metri avanti
    
    % Passo angolare per ogni step dell'MPC (quanto si sposta il target in 0.1s)
    step_theta = (V_target / R_circle) * Ts;
    
    for k = 1:N
        % L'angolo target 
        ang_k = current_arc_angle + look_ahead_angle + (k-1)*step_theta;
        
        ref_x = R_circle * cos(ang_k);
        ref_y = R_circle * sin(ang_k);
        
        % L'orientamento desiderato è la TANGENTE al cerchio in quel punto
        ref_theta = ang_k + pi/2;
        
        Ref_Horizon(1,k) = ref_x;
        Ref_Horizon(2,k) = ref_y;
        Ref_Horizon(3,k) = ref_theta;
        Ref_Horizon(4,k) = 0; % Sterzo preferibilmente a 0 
    end
    % Fix Start
    delta = x_current(3) - Ref_Horizon(3,1);
    delta_norm = atan2(sin(delta), cos(delta));
    x_mpc_input = x_current;
    x_mpc_input(3) = Ref_Horizon(3,1) + delta_norm;
    
    % Fix Orizzonte (Consistenza lungo N)
    base_angle = x_mpc_input(3);
    for k = 1:N
         diff = Ref_Horizon(3,k) - base_angle;
         diff = atan2(sin(diff), cos(diff));
         Ref_Horizon(3,k) = base_angle + diff;
         base_angle = Ref_Horizon(3,k);
    end
    
    y_ref_long = reshape(Ref_Horizon, [4*N, 1]);
    
    try
        [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev, y_ref_long, N, Ts, L, Q, R, ...
            u_min, u_max, du_min, du_max, C);
    catch
        u_opt = u_prev;
    end
    
    % Fisica Lineare
    [A_lin, B_lin, d_lin] = Linearized_Model(x_current, u_prev, Ts, L);
    
    % x(k+1) = A*x(k) + B*u(k) + d
    x_current = A_lin * x_current + B_lin * u_opt + d_lin;
    u_prev = u_opt;
    history_x = [history_x; x_current'];
end

%% VISUALIZZAZIONE
figure('Color','w', 'Name', 'Circle - Dynamic Horizon Fix');
hold on; grid on; axis equal;

% Cerchio
th = 0:0.01:2*pi;
plot(R_circle*cos(th), R_circle*sin(th), 'k--', 'LineWidth', 1.5);

% Traiettoria
plot(history_x(:,1), history_x(:,2), 'b-', 'LineWidth', 2);
plot(start_x, start_y, 'go', 'MarkerSize', 8, 'LineWidth', 2);

% Frecce direzione
step_vis = 30;
quiver(history_x(1:step_vis:end,1), history_x(1:step_vis:end,2), ...
       cos(history_x(1:step_vis:end,3)), sin(history_x(1:step_vis:end,3)), ...
       0.5, 'Color', 'r');

title('Traiettoria Corretta');
xlabel('X [m]'); ylabel('Y [m]');