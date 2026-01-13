function Ji = black_box_costo_2(x0,U,A,B,Q,R,S)
% A,B,Q,R,S sono le matrici reali
% m= dim input s = orizzonte temporale(righe di U)
[~,m] = size(B);
s = length(U);
N = s/m;
% definisco la matrice dei controlli U = [ u0; u1; u2 ecc]
U_matrice = reshape(U,m,N);
% Inizializzo costo e stato
Ji = 0;
x_k = x0;

for k = 1:N
    % controllo uk
    u_k = U_matrice(:,k);

    % Costo per lo stato k
    Ji = Ji + x_k'* Q * x_k + u_k' * R * u_k;

    % Calcolo stato successivo
    x_k = A * x_k + B * u_k;
end
%Fine finestra aggiungo il costo terminale S
Ji = Ji + x_k'* S * x_k;
end