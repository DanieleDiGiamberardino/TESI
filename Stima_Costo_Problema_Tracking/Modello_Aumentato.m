% Costruisco le matrici aumentate 
% Usiamo Delta_u come input, lo stato deve includere u(t-1)
% Nuovo Stato: x_aum = [x(t); u(t-1)]
% Nuovo Input: v(t) = Delta_u(t)
fprintf('Costruzione del modello aumentato\n');
%Implemento modello del doppio integratore
A = [ 1 1 ;
    0 1];
B = [0;
    1];
n = size(A, 1); % restituisce size delle righe
m = 1; % Dimensione dell'input
% Definisco le matrici del nuovo sistema
A_aum = [A, B;                % Blocco  [A, B]
         zeros(m,n), eye(m)]; % Blocco  [0, I]
B_aum = [B;               % Blocco in alto [B]
         eye(m)];               % Blocco in basso [I]

% DEFINISCO LA MATRICE DI USCITA 
C =[1, 0];

% NUOVA C
C_aum = [C , zeros(size(C,1),m)];

% Definisco la dimensione del nuovo stato
n_aum = n + m;
[p, ~] = size(C_aum); % Dimensione dell'uscita y

save('Modello_Aumentato.mat', 'A_aum', 'B_aum', 'C_aum', 'n', 'm', 'n_aum', 'p');
