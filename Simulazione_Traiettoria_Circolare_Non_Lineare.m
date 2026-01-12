%% SIMULAZIONE CERCHIO
clear all; clc; close all;

%% CONFIGURAZIONE
R_circle = 10; % Raggio cerchio 
V_target = 1.5; % Velocità target che vogliamo avere sul cerchio

% Partenza 
start_x = 20; start_y = -5; start_angle = pi; 
x_current = [start_x; start_y; start_angle; 0];

%% PARAMETRI
L = 1.2; Ts = 0.5; N = 6; % distanza tra le ruote, tempo campionamento, N orizzonti 
C = eye(4);

% Matrici di costo 
Q = diag([15, 15, 5, 0]); 
R = diag([5, 50]);       

% Vincoli sulla u assoluta
u_min = [-0.5; -0.6];
u_max = [ 3.0;  0.6]; 
% Vincoli sull'accelerazione/decelerazione
du_min = [-1.0; -0.2];
du_max = [ 1.0;  0.2];

%% SIMULAZIONE
u_prev = [0; 0]; % controllo iniziale
history_x = [];

fprintf('Avvio Simulazione\n');

% Variabili per gestire i giri
loops = 0;
prev_theta_car = atan2(start_y, start_x);

for t = 1:1500 

    theta_car = atan2(x_current(2), x_current(1)); % orinetamento attuale del veicolo range (-pi,pi)
    
    %% Gestione Wrapping Giri
    %(uso queste tecnica per evitare errori elevati tra theta_car e
    %theta_taget dovuti al range dell atan2)
    if (theta_car - prev_theta_car) < -5 % se la differenza tra Nuovo - Vecchio è <-5 allora o > 5 allora ho avuto in intera rotazione altrimenti 
        %normale variazione dell'angolo
        loops = loops + 1;
    elseif (theta_car - prev_theta_car) > 5 
        loops = loops - 1; 
    end
    % Salvo l angolo attuale come il precedente 
    prev_theta_car = theta_car;
    
    % Angolo assoluto 
    current_angle = theta_car + 2*pi*loops; % angolo anche > 2*Pi
    
    %% COSTRUZIONE ORIZZONTE
    % Calcoliamo N punti futuri sul cerchio
    Ref_Ori = zeros(4, N);
    
    % Distanza iniziale 
    look_ahead_angle = 2.0 / R_circle; % Guardiamo 2 metri avanti(è la distanza del primo punto che vogliamo raggiungere rispetto a dove mi trovo)
    
    % Passo angolare (quanto si sposta il target in 0.1s)
    step_theta = (V_target / R_circle) * Ts;
    
    for k = 1:N
        % L'angolo target 
        ang_k = current_angle + look_ahead_angle + (k-1)*step_theta;
        
        % Mi calcolo le coordinate del punto k-esimo che voglio raggiungere
        % sulla circonferenza 
        ref_x = R_circle * cos(ang_k);
        ref_y = R_circle * sin(ang_k);
        
        % L'orientamento desiderato TANGENTE al cerchio in quel punto
        ref_theta = ang_k + pi/2;
        
        % Definisco i valori del k-esimo punto che voglio raggiungere 
        Ref_Ori(1,k) = ref_x;
        Ref_Ori(2,k) = ref_y;
        Ref_Ori(3,k) = ref_theta;
        Ref_Ori(4,k) = atan(L/R_circle);% Sterzo 
    end
    %% Fix Start ANGOLO
    delta = x_current(3) - Ref_Ori(3,1);
    delta_norm = atan2(sin(delta), cos(delta));
    x_mpc_input = x_current;
    x_mpc_input(3) = Ref_Ori(3,1) + delta_norm; % ho creato uno stato fittizio x_mpc_input in modo tale che passo al MPC 
    % Ref + errore_piccolo questo "inganna" l'MPC facendogli vedere l'auto vicinissima al riferimento. 
    
    %% Fix Orizzonte ANGOLO -> facciamo la stessa cosa di sopra ma su tutto
    % l'orrizonte della finestra sul riferimento stesso problema 
    %Se la differenza è il salto tra 179 e -179 (cioè 358), questa formula restituisce 2.
    base_angle = x_mpc_input(3);
    for k = 1:N
         diff = Ref_Ori(3,k) - base_angle;
         diff = atan2(sin(diff), cos(diff));
         Ref_Ori(3,k) = base_angle + diff;
         base_angle = Ref_Ori(3,k);
    end
    
    y_ref_long = reshape(Ref_Ori, [4*N, 1]); % sono i vari punti che mi sono costruito un vettore colonna
    
    try
        [u_opt, ~] = MPC_Linear(x_mpc_input, u_prev, y_ref_long, N, Ts, L, Q, R, u_min, u_max, du_min, du_max, C);
    catch
        u_opt = u_prev;
    end
    
    %% Fisica non lineare 
    dxdt = Car_Like_Model(x_current, u_opt, L);
    x_current = x_current + dxdt * Ts;
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

title('Traiettoria ');
xlabel('X [m]'); ylabel('Y [m]');