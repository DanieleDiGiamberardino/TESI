function [u_ottimo, J_val] = Solve_CarLike(x0, x_target, N, Ts, L, Q, R)
    % RISOLUTORE NMPC (Nonlinear Model Predictive Control)  
    n_inputs = 2; % [v, w]
    
    % Initial Guess (Tentativo iniziale)
    % Partiamo ipotizzando di stare fermi (tutti zeri)
    u_guess = zeros(N * n_inputs, 1);
    
    % Vincoli Fisici 
    % v:  tra -2 m/s e +2 m/s
    % w:  tra -1 rad/s e +1 rad/s
    lb_single = [-2; -1]; 
    ub_single = [ 2;  1];
    
    % Ripetiamo i vincoli per tutto l'orizzonte N
    lb = repmat(lb_single, N, 1);
    ub = repmat(ub_single, N, 1);
    
    % Opzioni Ottimizzatore (fmincon)
    % 'sqp' è un algoritmo veloce per problemi non lineari
    options = optimoptions('fmincon', 'Display', 'off', ...
        'Algorithm', 'sqp', 'MaxIterations', 100);
    
    % Definizione Funzione Costo
    % Creiamo una "funzione anonima" che fmincon può chiamare.
    % Gli passiamo solo U, gli altri parametri sono fissati.
    funzione_costo = @(U) internal_cost(U, x0, x_target, N, Ts, L, Q, R);
    
    % RISOLUZIONE DEL PROBLEMA (Qui avviene la magia lenta)
    [U_seq_opt, J_val] = fmincon(funzione_costo, u_guess, [], [], [], [], lb, ub, [], options);
    
    % Estrazione Risultato ( prendiamo solo il primo controllo ottimo
    % calcolato)
    u_ottimo = U_seq_opt(1:n_inputs);
end

% SOTTO-FUNZIONE CHE CALCOLA IL COSTO J 
function J = internal_cost(U_seq, x0, x_ref, N, Ts, L, Q, R)
    % Ricostruisce la matrice degli input (2 righe, N colonne)
    U_mat = reshape(U_seq, 2, N);
    
    x_curr = x0;
    J = 0;
    
    % Simulazione nel futuro (Predizione)
    for k = 1:N
        u_k = U_mat(:, k);
        
        % Integrazione numerica (Eulero) per trovare x(k+1)
        dxdt = Car_Like_Model(0, x_curr, u_k, L);
        x_next = x_curr + dxdt * Ts;
        
        % Calcolo Errore rispetto al target
        error = x_next - x_ref;
        
        % J = Errore^2 + Sforzo^2
        J = J + (error' * Q * error) + (u_k' * R * u_k);
        
        % Aggiorna stato per il passo dopo
        x_curr = x_next;
    end
end