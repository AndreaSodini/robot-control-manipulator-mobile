FS_title  = 15;
FS_label  = 13;
FS_tick   = 11;
FS_legend = 11;
LW_thin   = 1.0;

N_JOINTS  = 6;
N_PARAMS  = 7;

PNAMES = {'J', 'c_2', 'b', 'k', 'g_s', 'g_c', 'f_c'};

cmap6 = lines(N_JOINTS);
cmap7 = lines(N_PARAMS);

th_raw  = out.theta_hat.Data;
t_adapt = out.theta_hat.Time;

th_raw = squeeze(th_raw);
if size(th_raw,1) == length(t_adapt)
    theta_data = th_raw';       
else
    theta_data = th_raw;
end


%% FIGURA — Evoluzione theta_hat nel tempo per giunto

figure('Name','Fig 1: Convergenza parametri dinamici per giunto','Position',[40 40 1500 950]);
sgtitle('Evoluzione temporale parametri dinamici per giunto','FontSize', FS_title,'FontWeight', 'bold')
for j = 1:N_JOINTS
    subplot(3,2,j)
    base = (j-1)*N_PARAMS;
    hold on
    for p = 1:N_PARAMS
        idx = base + p;
        plot(t_adapt, theta_data(idx,:), '-', ...
             'Color', cmap7(p,:), 'LineWidth', LW_thin, ...
             'DisplayName', PNAMES{p});
    end
    xline(t_adapt(round(0.8*end)), '--k', 'LineWidth', 0.8, ...
          'Label','regime','FontSize',8,'HandleVisibility','off');
    grid on
    xlabel('Time [s]',     'FontSize', FS_label-1)
    ylabel('\hat{\theta}', 'FontSize', FS_label-1)
    title(sprintf('Giunto %d', j), 'FontSize', FS_title-2)
    legend('Location','eastoutside','FontSize',FS_legend-3,'NumColumns',1)
    set(gca,'FontSize',FS_tick)
end


