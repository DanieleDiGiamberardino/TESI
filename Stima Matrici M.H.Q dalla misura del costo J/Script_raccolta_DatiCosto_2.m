% Definiamo lo script per raccogliere i dati
marg = 60;
% Inizializzo i parametri veri
A = [1 1 ; 0 1];
B = [0 ; 1];
Q_s = [0.2 0 ;
    0 0];
R_s = 1;
S = Q_s;
[n,~] = size(A);
[~,m] = size(B);

% Inizializzo l'orizzonte temporale
N = 5;
s = m * N;

% Dato che devo eseguire almeno tante misure quanti sono i parametri
% sconosciuti mi calcoli la loro dimensione a priori
p_M = s*s;
p_H = s*n;
p_Q = n*n;

Num_min = p_Q + p_H + p_M;
Num_esp = Num_min + marg;

% Settaggio livelli di rumore 

% Rumore sui sensori
delta_x0 = 0.05; % questo valore è solo relativo non implica un errore in % perchè dipende dal valore di randn()
delta_U = 0.05;  % questo valore è solo relativo non implica un errore in % perchè dipende dal valore di randn()

% Rumore sulla Misura
delta_J = 0.02;  % questo valore è solo relativo non implica un errore in % perchè dipende dal valore di randn()

% Matrici per la regressione lineare
Y_costo = zeros(Num_esp,1);
Phi = zeros(Num_esp,Num_min);

for i = 1:Num_esp

    x0_i_vero = randn(n,1); % Stati casuale ( definisco x01 e x02)
    U_i_vero = randn(s,1); % Sequenza di controllo casuale

    % Calcolo il costo attraverso la black_box (VERA)
    J_i_vero = black_box_costo_2(x0_i_vero,U_i_vero,A,B,Q_s,R_s,S);

    % Aggiungo qui il rumore quindi facciamo una prima misura VERA
    
    % Rumore additivo su x0 ("deltax")
    deltax = delta_x0 * randn(n,1); 
    x0_i_mis = x0_i_vero + deltax;

    % Rumore additivo su U
    deltaU = delta_U * randn(s,1); 
    U_i_mis = U_i_vero + deltaU;

    % Rumore moltiplicativo su J
    deltaJ = delta_J * J_i_vero * randn();
    J_i_mis = J_i_vero + deltaJ;

    % Costruisco il regressore
    phi_M = kron(U_i_mis',U_i_mis');
    phi_H = 2*kron(x0_i_mis',U_i_mis');
    phi_Q = kron(x0_i_mis',x0_i_mis');

    phi_i = [phi_M , phi_H , phi_Q];

    % Salvo i valori
    Y_costo(i) = J_i_mis; % Salvo J rumoroso
    Phi(i,:) = phi_i; % Salvo Phi rumoroso
end
fprintf("Dati Raccolti END");
save("Script_raccolta_DatiCosto_2.mat","Y_costo","Phi","n","m","N");



