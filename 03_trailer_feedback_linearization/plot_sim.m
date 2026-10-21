% Parametri temporali
dt = 0.01;
t = (0:length(err)-1) * dt;

%% --- Plot Errore ---
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

%% --- Plot 4 stati ---
dt = 0.01;
t = (0:size(X_sim,2)-1) * dt;

figure('Color','w');

% 1) Posizione x rimorchio
subplot(2,2,1);
plot(t, X_sim(1,:), 'b-', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 14);
ylabel('x [m]', 'FontSize', 14);
title('Posizione x rimorchio', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 13, 'GridAlpha', 0.4);

% 2) Posizione y rimorchio
subplot(2,2,2);
plot(t, X_sim(2,:), 'r-', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 14);
ylabel('y [m]', 'FontSize', 14);
title('Posizione y rimorchio', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 13, 'GridAlpha', 0.4);

% 3) Angolo assoluto rimorchio theta
subplot(2,2,3);
plot(t, rad2deg(X_sim(3,:)), 'g-', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 14);
ylabel('\theta [°]', 'FontSize', 14);
title('Angolo assoluto rimorchio \theta', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 13, 'GridAlpha', 0.4);

% 4) Angolo relativo phi
subplot(2,2,4);
plot(t, rad2deg(X_sim(4,:)), 'm-', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 14);
ylabel('\phi [°]', 'FontSize', 14);
title('Angolo relativo biciclo-rimorchio \phi', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 13, 'GridAlpha', 0.4);

sgtitle('Stati del sistema', 'FontSize', 17, 'FontWeight', 'bold');

%% Plot Ingressi
u = squeeze(U_sim);
vel = u(1,:);
delta = u(2,:);

figure;

% --- Velocità ---
subplot(1,2,1);
plot(t, vel, 'b-', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 14);
ylabel('Velocità [m/s]', 'FontSize', 14);
title('Ingresso: Velocità', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 13, 'GridAlpha', 0.4);

% --- Angolo di sterzo delta ---
subplot(1,2,2);
plot(t, delta, 'r-', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 14);
ylabel('\delta [rad]', 'FontSize', 14);
title('Ingresso: Angolo di sterzata \delta', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 13, 'GridAlpha', 0.4);

%% --- Plot v_t ---
v_t = simOut.v_t;
figure;
plot(t, v_t, 'b-', 'LineWidth', 2);
xlabel('Tempo [s]', 'FontSize', 14);
ylabel('Velocità [m/s]', 'FontSize', 14);
title('Ingresso: Velocità', 'FontSize', 15, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 13, 'GridAlpha', 0.4);