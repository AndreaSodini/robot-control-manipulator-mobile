clear all
clc
close all

model_name = 'bicycle_trailer_trailer';
T_end      = 60;

%% Parametri sistema
L = 1.5;    % interasse biciclo [m]
M = 1;      % interasse rimorchio [m]
d = 0.55;   % offset punto virtuale davanti all'asse post. rimorchio [m]

% Traiettoria riferimento cerchio
R_ref = 3.0;
w_ref = 0.25;
cx    = 0.0;
cy    = 0.0;

% Stato iniziale
X0 = [5;            % x_t
      8;            % y_t
      1.67079633;   % theta_t
      0.08];        % phi 

%% Controllo LQR
A_lqr = [zeros(2) eye(2);
         zeros(2) zeros(2)];
B_lqr = [zeros(2);
         eye(2)];

e_lqr = 1e-3;
Q = eye(4)*1/e_lqr;
R = eye(2)*1/e_lqr;
K = lqr(A_lqr,B_lqr,Q,R);

%% Simulazione
anim_fps   = 60;
anim_speed = 1.0;

tic;
simOut = sim(model_name);
t_sim  = toc;
fprintf('  Completata in %.2f s\n', t_sim);

%% Variabili per Plot e Animazione
X_sim   = simOut.X_sim;
U_sim   = simOut.U_sim;
Ref_sim = simOut.Ref_sim;
T_sim   = simOut.T_sim;

% Uniforma dimensioni: [4xN], [2xN], [6xN], [1xN]
if size(X_sim,2)   == 4, X_sim   = X_sim';  end
if size(U_sim,2)   == 2, U_sim   = U_sim';  end
if size(Ref_sim,2) == 6, Ref_sim = Ref_sim'; end
T_sim = T_sim(:)';

N       = size(X_sim, 2);
x_t     = X_sim(1,:);   % asse posteriore rimorchio
y_t     = X_sim(2,:);
theta_t = X_sim(3,:);   % orientamento rimorchio
phi     = X_sim(4,:);   % angolo relativo biciclo-rimorchio

% Ricostruisci posizione asse posteriore biciclo (gancio)
% Il gancio è a distanza M davanti all'asse posteriore del rimorchio
x_h = x_t + M*cos(theta_t);   % posizione gancio = asse post. biciclo
y_h = y_t + M*sin(theta_t);

% Orientamento biciclo
theta_b = theta_t + phi;

% Asse anteriore biciclo
x_f = x_h + L*cos(theta_b);
y_f = y_h + L*sin(theta_b);

% Riferimento (sull'asse posteriore rimorchio)
xr_ref = Ref_sim(1,:);
yr_ref = Ref_sim(2,:);

% Errore di tracking sull'asse posteriore rimorchio
err = sqrt((x_t - xr_ref).^2 + (y_t - yr_ref).^2);


