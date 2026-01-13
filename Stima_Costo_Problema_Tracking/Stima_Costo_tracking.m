% STIMA DELLE MATRICI DI COSTO
fprintf("\n Avvio Stima Costo per TRACKING \n");

load("DatiCosto_TRACKING.mat");

P_stima = pinv(Phi) * Y_costo;

% Ricostruisco le matrici 
dim_vec_Msy = s * s;
dim_vec_Hsy = s * n_aum;
dim_vec_Qsy = n_aum * n_aum;
dim_vec_vu  = p * s;
dim_vec_vx  = p * n_aum;
dim_vec_qr  = p * p;
% Definisco indici per estrarre i valori dalla Phi
idx_start_Msy = 1;
idx_end_Msy   = dim_vec_Msy;

idx_start_Hsy = idx_end_Msy + 1;
idx_end_Hsy   = idx_end_Msy + dim_vec_Hsy;

idx_start_Qsy = idx_end_Hsy + 1;
idx_end_Qsy   = idx_end_Hsy + dim_vec_Qsy;

idx_start_vu  = idx_end_Qsy + 1;
idx_end_vu    = idx_end_Qsy + dim_vec_vu;

idx_start_vx  = idx_end_vu + 1;
idx_end_vx    = idx_end_vu + dim_vec_vx;

idx_start_qr  = idx_end_vx + 1;
idx_end_qr    = idx_end_vx + dim_vec_qr;

% Estraggo i vettori Msy Hsy Qsy vu vx qr
vec_M = P_stima(idx_start_Msy : idx_end_Msy);
vec_H = P_stima(idx_start_Hsy : idx_end_Hsy);
vec_Qb = P_stima(idx_start_Qsy : idx_end_Qsy);
vu_stima = P_stima(idx_start_vu : idx_end_vu);
vx_stima = P_stima(idx_start_vx : idx_end_vx);
qr_stima = P_stima(idx_start_qr : idx_end_qr);

% Reshape in matrici
Msy_stima = reshape(vec_M, s, s);
Hsy_stima = reshape(vec_H, s, n_aum);
Qsy_stima = reshape(vec_Qb, n_aum, n_aum);
vu_stima = reshape(vu_stima, s, p);

% Salvo i dati
save("Matrici_Stimate_TRACKING.mat", "Msy_stima", "Hsy_stima", "Qsy_stima","vu_stima","vx_stima","qr_stima");

fprintf('Matrici salvate\n');

