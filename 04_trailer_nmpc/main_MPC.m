clear; clc; close all;

%% ---- Parametri sistema ----
p.L         = 1.5;          % interasse biciclo [m]
p.M         = 1.0;          % braccio rimorchio [m]
p.dt        = 0.05;         % passo integrazione [s]
p.phi_max   = deg2rad(60);  % limite jackknife [rad]
p.delta_max = deg2rad(35);  % limite sterzo [rad]


%%  TASK 1: Inseguimento Traiettoria Circolare
fprintf(' TASK 1: Inseguimento Traiettoria Circolare\n');

R_c   = 3.0;
w_ref = 0.25;
T1    = 60;
t1    = 0:p.dt:T1;
N1    = length(t1);

alpha1 = w_ref * (0:N1-1) * p.dt;
x_ref1 = [R_c*cos(alpha1);
          R_c*sin(alpha1);
          atan2(cos(alpha1), -sin(alpha1));
          zeros(1,N1)];

v_ff     = R_c * w_ref;
phi_ss   = asin(min(0.95, w_ref*p.M/v_ff));
v_ff     = min(2.0, max(0.1, R_c*w_ref/cos(phi_ss)));
phi_ss   = asin(min(0.95, w_ref*p.M/v_ff));
delta_ff = atan(p.L*tan(phi_ss));
u_ref1   = repmat([v_ff; delta_ff], 1, N1);
x_ref1(4,:) = phi_ss;

mpc1.N     = 45;
mpc1.Nc    = 8;
mpc1.Q     = diag([10,   10,   10, 0]);
mpc1.Qf    = mpc1.Q;
mpc1.R     = diag([ 0.5,  1]);
mpc1.R_du  = diag([ 1.5,  2]);
mpc1.u_min = [-1; -p.delta_max];
mpc1.u_max = [ 3.0;  p.delta_max];

x0_1 = [5; 8; 1.67079633; 0.08];
[X1,U1] = mpc_simulate(x0_1, x_ref1, u_ref1, t1, mpc1, p);
animate_system(X1, U1, t1, x_ref1, p, 'Task 1 - Inseguimento Circolare', 'task1_animation.gif', []);
plot_results(X1, U1, t1, x_ref1, u_ref1, 'Traiettoria Circolare');

% Vettore errore Task 1
e_x1     = X1(1,:) - x_ref1(1,:);
e_y1     = X1(2,:) - x_ref1(2,:);
e_theta1 = atan2(sin(X1(3,:) - x_ref1(3,:)), cos(X1(3,:) - x_ref1(3,:)));
e_phi1   = atan2(sin(X1(4,:) - x_ref1(4,:)), cos(X1(4,:) - x_ref1(4,:)));
err   = sqrt(e_x1.^2 + e_y1.^2);   % errore posizione 

% Parametri temporali
t = (0:length(err)-1) * p.dt;

% Plot Errore
figure;
ax2 = gca;
fill([t, fliplr(t)], [err, zeros(1, size(err,2))], ...
    [0.68 0.85 0.90], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
hold on;
plot(t, err, 'b-', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 14);
ylabel('Errore [m]', 'FontSize', 14);
title('Errore di Tracking', 'FontSize', 16, 'FontWeight', 'bold');
lg2 = legend('Errore', 'Location', 'best');
lg2.FontSize = 13;
grid on;
ax2.FontSize = 13;
set(ax2, 'GridAlpha', 0.4);
xlim([0, t(end)]);
ylim([0, max(err)*1.05]);
hold off;

%% TASK 2: Inseguimento otto
fprintf(' TASK 2: Inseguimento Figura 8\n');

R_8  = 4.0;     % raggio di ogni cerchio [m]
v_8  = 0.8;     % velocita' costante [m/s]
T2   = 60;      % ~2.5 giri completi
t2   = 0:p.dt:T2;
N2   = length(t2);

% Parametro angolare:
T_half = pi * R_8 / v_8;      % durata di ogni cerchio = 15.71 s
w_8    = 2*pi / T_half;       

% Costruzione riferimento punto per punto
x_ref2    = zeros(4, N2);
u_ref2    = zeros(2, N2);

% Feedforward analitico costante per ogni cerchio
phi_ccw   =  asin( 1/R_8 * p.M);   % +22.0 deg
phi_cw    =  asin(-1/R_8 * p.M);   % -22.0 deg
delta_ccw =  atan(p.L * tan( phi_ccw));
delta_cw  =  atan(p.L * tan( phi_cw));

for k = 1:N2
    s     = w_8 * (k-1) * p.dt;          % angolo percorso [rad]
    s_mod = mod(s, 4*pi);                % [0, 4*pi)

    if s_mod < 2*pi
        a = pi + s_mod;
        x_ref2(1,k) = R_8 * (1 + cos(a));
        x_ref2(2,k) = R_8 * sin(a);
        x_ref2(3,k) = a + pi/2;             
        x_ref2(4,k) = phi_ccw;
        u_ref2(1,k) = v_8;
        u_ref2(2,k) = delta_ccw;
    else
        % Cerchio 2: CW, centro (-R,0)
        b = s_mod - 2*pi;                     % b: [0, 2*pi)
        x_ref2(1,k) = R_8 * (cos(b) - 1);
        x_ref2(2,k) = -R_8 * sin(b);
        x_ref2(3,k) = atan2(-cos(b), -sin(b));  
        x_ref2(4,k) = phi_cw;
        u_ref2(1,k) = v_8;
        u_ref2(2,k) = delta_cw;
    end
end

% Wrap heading in [-pi, pi]
x_ref2(3,:) = atan2(sin(x_ref2(3,:)), cos(x_ref2(3,:)));

% Stato iniziale: sull'origine con piccola perturbazione
x0_2 = [x_ref2(1,1) + 0.2;
         x_ref2(2,1) - 0.3;
         x_ref2(3,1) + deg2rad(8);
         0];

% Parametri MPC
mpc2.N     = 30;
mpc2.Nc    = 8;
mpc2.Q     = diag([10,   10,   10, 0]);
mpc2.Qf    = mpc2.Q;
mpc2.R     = diag([ 0.5,  1]);
mpc2.R_du  = diag([ 1.5,  2]);
mpc2.u_min = [-1; -p.delta_max];
mpc2.u_max = [ 3.0;  p.delta_max];

[X2,U2] = mpc_simulate(x0_2, x_ref2, u_ref2, t2, mpc2, p);
animate_system(X2, U2, t2, x_ref2, p, ...
    'Task 2 - Inseguimento Figura 8', 'task2_animation.gif', []);
plot_results(X2, U2, t2, x_ref2, u_ref2, 'Traiettoria Lemniscata');

