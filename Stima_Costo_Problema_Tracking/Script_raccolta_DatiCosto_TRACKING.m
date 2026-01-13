clear all; clc;
fprintf('\nAvvio Raccolta Dati per il TRACKING\n');

% Carico il modello aumentato
load("Modello_Aumentato.mat");

% Definisco i valori veri del costo
Q_y_vera = 1;      % Penalità sull'errore di tracking
R_delta_vera = 10; % Penalità sugli incrementi Delta_U

marg = 100;
N = 5;     % Orizzonte
s = m * N; % Dimensione del vettore z
p = N;

% Calcolo il num di parametri da stimare
p_Msy = s * s;                    % Matrice M_sy
p_Hsy = s * n_aum;                % Matrice H_sy
p_Qsy = n_aum * n_aum;            % Matrice Q_sy
p_vu  = p*s;                        % Vettore v_u
p_vx  = p * n_aum;                    % Vettore v_x
p_qr  = p * p;

Num_min = p_Msy + p_Hsy + p_Qsy + p_vu + p_vx + p_qr;
Num_esp = Num_min + marg;

% Inizializzo matrici per la raccolta dati
Y_costo = zeros(Num_esp,1);
Phi = zeros(Num_esp, Num_min);

% Ciclo per la raccolta dei dati
for i = 1:Num_esp
    % Inizializzo stato aumentato Casuale
    x0_i_vero = randn(n,1);      % Stato fisico iniziale
    u_prev_vero = randn(m,1);  % Controllo al passo precedente
    r_i_vero = randn(p,1);       % Riferimento da inseguire
    Delta_U_vero = randn(s,1); % Genero una sequenza casuale di INCREMENTI di controllo

    % Calcolo il costo vero associato alla sequenza random Delta_U allo
    % stato x0 e al riferimento ri
    J_i_vero = black_box_costo_tracking(x0_i_vero,u_prev_vero,Delta_U_vero,r_i_vero,A_aum,B_aum,C_aum,Q_y_vera,R_delta_vera);

    % Rumore
    deltax = 0.01 * randn(n,1);
    x0_i = x0_i_vero + deltax;

    deltau_prev = 0.01 * randn(m,1);
    u_prev_i = u_prev_vero + deltau_prev;

    delta_seq = 0.02 * randn(s,1);
    Delta_U_i = Delta_U_vero + delta_seq;

    deltaJ = 0.01 * J_i_vero * randn();
    J_i_mis = J_i_vero + deltaJ;

    deltari = 0.01 * randn(p,1);
    r_i = r_i_vero + deltari;

    % Definisco la stato aumentato
    x_tilde = [x0_i; u_prev_i];

    % Parte che dipende solo da xo_tilde e Delta_U
    phi_Msy = kron(Delta_U_i', Delta_U_i');
    phi_Hsy = 2 * kron(x_tilde',Delta_U_i');
    phi_Qsy = kron(x_tilde',x_tilde');

    phi_vu = -2 * kron(r_i', Delta_U_i');
    phi_vx = -2 * kron(r_i', x_tilde');
    phi_qr = kron(r_i', r_i');

    Phi_i = [phi_Msy , phi_Hsy, phi_Qsy, phi_vu, phi_vx, phi_qr];

    % Salvo i dati
    Y_costo(i) = J_i_mis;
    Phi(i,:) = Phi_i;
end

fprintf('Raccolta dati completata.\n');
save('DatiCosto_TRACKING.mat', 'Y_costo', 'Phi', 'n', 'm', 'N', 's', 'n_aum');



