% Caricamento del modello del robot ABB IRB120
robot = loadrobot("abbIrb120", "DataFormat", "column");
ee = robot.Bodies{end}.Name;

q      = out.q.Data;
q_real = out.q_real.Data;
t      = out.q.Time;
xi_d   = (out.xid.Data)';
q_d    = squeeze(out.q_d.Data).';

% Inizializzazioni
p     = zeros(3, size(q,1));
p(:,1) = ee0(1:3);

p_real = zeros(3, size(q_real,1));
p_real(:,1) = ee0_real(1:3);

p_d   = zeros(3, size(q_d,1));
p_d(:,1) = [0.558; 0; 0.426];

% FK
for i = 1:(size(q,1)-1)
    Ti = getTransform(robot, q(i,:)', ee);
    p(:,i+1) = Ti(1:3,4);
end

for i = 1:(size(q_real,1)-1)
    Ti_real = getTransform(robot, q_real(i,:)', ee_r);
    p_real(:,i+1) = Ti_real(1:3,4);
end

for i = 1:(size(q_d,1)-1)
    Ti = getTransform(robot, q_d(i,:)', ee);
    p_d(:,i+1) = Ti(1:3,4);
end

colors    = {'r','g','b'};
FS_title  = 16;   % FontSize titoli
FS_label  = 14;   % FontSize label assi
FS_tick   = 12;   % FontSize tick
FS_legend = 12;   % FontSize legenda
LW_ref    = 2.0;  % LineWidth riferimento
LW_ctrl   = 1.5;  % LineWidth segnale controllato

%% FIGURA 1
figure('Name','Fig1: Imposto vs Inversione IK')
h = gobjects(6,1);
for k = 1:3
    h(k)   = plot(t, xi_d(k,:), '--', 'Color', colors{k}, 'LineWidth', LW_ref);  hold on
    h(k+3) = plot(t, p_d(k,:),  '-',  'Color', colors{k}, 'LineWidth', LW_ctrl);
end
grid on
xlabel('Time [s]',     'FontSize', FS_label)
ylabel('Position [m]', 'FontSize', FS_label)
title('Confronto: Traiettoria Imposta vs Inversione Cinematica (IK)', 'FontSize', FS_title)
legend(h, 'X_{d,x}','X_{d,y}','X_{d,z}', ...
          'p_{d,x} (IK)','p_{d,y} (IK)','p_{d,z} (IK)', ...
          'Location','best', 'FontSize', FS_legend)
set(gca, 'FontSize', FS_tick)

%% FIGURA 2 
figure('Name','Fig2: Imposto vs EE BS')
h = gobjects(6,1);
for k = 1:3
    h(k)   = plot(t, xi_d(k,:), '--', 'Color', colors{k}, 'LineWidth', LW_ref);  hold on
    h(k+3) = plot(t, p(k,:),    '-',  'Color', colors{k}, 'LineWidth', LW_ctrl);
end
grid on
xlabel('Time [s]',     'FontSize', FS_label)
ylabel('Position [m]', 'FontSize', FS_label)
title('Confronto: Traiettoria Imposta vs Posizione EE BS', 'FontSize', FS_title)
legend(h, 'X_{d,x}','X_{d,y}','X_{d,z}', ...
          'p_x','p_y','p_z', ...
          'Location','best', 'FontSize', FS_legend)
set(gca, 'FontSize', FS_tick)

%% FIGURA 3 
figure('Name','Fig3: Imposto vs EE CT ADATTIVO con INCERTEZZE')
h = gobjects(6,1);
for k = 1:3
    h(k)   = plot(t, xi_d(k,:),   '--', 'Color', colors{k}, 'LineWidth', LW_ref);  hold on
    h(k+3) = plot(t, p_real(k,:), ':',  'Color', colors{k}, 'LineWidth', LW_ref);
end
grid on
xlabel('Time [s]',     'FontSize', FS_label)
ylabel('Position [m]', 'FontSize', FS_label)
title('Confronto: Traiettoria Imposta vs Posizione EE CT ADATTIVO con Incertezze', 'FontSize', FS_title)
legend(h, 'X_{d,x}','X_{d,y}','X_{d,z}', ...
          'p_x (reale)','p_y (reale)','p_z (reale)', ...
          'Location','best', 'FontSize', FS_legend)
set(gca, 'FontSize', FS_tick)

%% FIGURA 4
figure('Name', 'Confronto q_d vs q_inc', 'Position', [100 100 1400 900])

joint_names = {'Giunto 1','Giunto 2','Giunto 3', ...
               'Giunto 4','Giunto 5','Giunto 6'};

for k = 1:6
    subplot(3, 2, k)

    plot(t, q_d(:,k),   '--r', 'LineWidth', LW_ref,  'DisplayName', 'q_d (desiderato)'); hold on
    plot(t, q_real(:,k), '-b', 'LineWidth', LW_ctrl, 'DisplayName', 'q (controllato)');

    yyaxis right
    plot(t, q_d(:,k) - q_real(:,k), '-', 'Color', [0.6 0.6 0.6], ...
         'LineWidth', 0.8, 'DisplayName', 'errore');
    ylabel('Errore [rad]', 'FontSize', FS_label, 'Color', [0.4 0.4 0.4])
    ax = gca;
    ax.YAxis(2).Color = [0.4 0.4 0.4];
    yyaxis left

    grid on
    xlabel('Time [s]',       'FontSize', FS_label)
    ylabel('Posizione [rad]','FontSize', FS_label)
    title(joint_names{k},    'FontSize', FS_title - 2)
    legend('Location','best','FontSize', FS_legend - 1)
    set(gca, 'FontSize', FS_tick)
end

sgtitle('Confronto Configurazioni di Giunto CT con Incertezze: Desiderato vs Controllato', ...
        'FontSize', FS_title + 2, 'FontWeight', 'bold')

%% FIGURA 5: 
figure('Name', 'Confronto q_d vs q_real', 'Position', [100 100 1400 900])

joint_names = {'Giunto 1','Giunto 2','Giunto 3', ...
               'Giunto 4','Giunto 5','Giunto 6'};

for k = 1:6
    subplot(3, 2, k)

    plot(t, q_d(:,k),   '--r', 'LineWidth', LW_ref,  'DisplayName', 'q_d (desiderato)'); hold on
    plot(t, q(:,k), '-b', 'LineWidth', LW_ctrl, 'DisplayName', 'q (controllato)');

    yyaxis right
    plot(t, q_d(:,k) - q(:,k), '-', 'Color', [0.6 0.6 0.6], ...
         'LineWidth', 0.8, 'DisplayName', 'errore');
    ylabel('Errore [rad]', 'FontSize', FS_label, 'Color', [0.4 0.4 0.4])
    ax = gca;
    ax.YAxis(2).Color = [0.4 0.4 0.4];
    yyaxis left

    grid on
    xlabel('Time [s]',       'FontSize', FS_label)
    ylabel('Posizione [rad]','FontSize', FS_label)
    title(joint_names{k},    'FontSize', FS_title - 2)
    legend('Location','best','FontSize', FS_legend - 1)
    set(gca, 'FontSize', FS_tick)
end

sgtitle('Confronto Configurazioni di Giunto: Desiderato vs Controllato', ...
        'FontSize', FS_title + 2, 'FontWeight', 'bold')

%% FIGURA 6: 
valcondY = out.condY.data;

figure;
semilogy(valcondY, 'o-');
grid on;
title('Valori singolari del regressore');
xlabel('Indice');
ylabel('Valore singolare');