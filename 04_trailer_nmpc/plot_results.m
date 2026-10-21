function plot_results(X, U, t, x_ref, u_ref, fig_title)
    Nsim = size(U,2);
    dt   = t(2)-t(1);
    tx   = (0:size(X,2)-1)*dt;
    tu   = (0:Nsim-1)*dt;

    %%  FIGURA 1 — Traiettoria XY
    figure('Color','w','Name',[fig_title ' – Traiettoria']);

    hold on; grid on; axis equal;
    is_static = isequal(x_ref(:,1), x_ref(:,end));
    if is_static
        plot(x_ref(1,1), x_ref(2,1), 'p', 'MarkerSize', 14,'Color', [0 0.7 0], 'MarkerFaceColor', [0 0.7 0], 'DisplayName', 'Target');
    else
        plot(x_ref(1,:), x_ref(2,:), '--r', 'LineWidth', 1.8, 'DisplayName', 'Riferimento circolare');
    end
    plot(X(1,:), X(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Traccia asse post. rimorchio');
    plot(X(1,1),  X(2,1),  'go', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot(X(1,end),X(2,end),'rs', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'r', 'DisplayName', 'End');

    xlabel('x [m]',  'FontSize', 14);
    ylabel('y [m]',  'FontSize', 14);
    title([fig_title ' – Traiettoria XY'], 'FontSize', 16, 'FontWeight', 'bold');
    lg = legend('Location', 'best');
    lg.FontSize = 13;
    set(gca, 'FontSize', 13, 'GridAlpha', 0.4);
    hold off;

    %% FIGURA 2 — Stati del sistema
    figure('Color','w','Name',[fig_title ' – Stati']);

    % 1) Posizione x
    subplot(2,2,1);
    plot(tx, X(1,:), 'b-', 'LineWidth', 2); hold on;
    plot(tx, x_ref(1,1:length(tx)), '--k', 'LineWidth', 1.2);
    xlabel('Tempo [s]', 'FontSize', 14);
    ylabel('x [m]',    'FontSize', 14);
    title('Posizione x rimorchio', 'FontSize', 15, 'FontWeight', 'bold');
    legend('Reale','Rif.','Location','best','FontSize',12);
    grid on; set(gca,'FontSize',13,'GridAlpha',0.4); hold off;

    % 2) Posizione y
    subplot(2,2,2);
    plot(tx, X(2,:), 'r-', 'LineWidth', 2); hold on;
    plot(tx, x_ref(2,1:length(tx)), '--k', 'LineWidth', 1.2);
    xlabel('Tempo [s]', 'FontSize', 14);
    ylabel('y [m]',    'FontSize', 14);
    title('Posizione y rimorchio', 'FontSize', 15, 'FontWeight', 'bold');
    legend('Reale','Rif.','Location','best','FontSize',12);
    grid on; set(gca,'FontSize',13,'GridAlpha',0.4); hold off;

    % 3) Angolo assoluto theta
    subplot(2,2,3);
    plot(tx, rad2deg(X(3,:)), 'g-', 'LineWidth', 2); hold on;
    plot(tx, rad2deg(x_ref(3,1:length(tx))), '--k', 'LineWidth', 1.2);
    xlabel('Tempo [s]',   'FontSize', 14);
    ylabel('\theta [°]',  'FontSize', 14);
    title('Angolo assoluto rimorchio \theta', 'FontSize', 15, 'FontWeight', 'bold');
    legend('Reale','Rif.','Location','best','FontSize',12);
    grid on; set(gca,'FontSize',13,'GridAlpha',0.4); hold off;

    % 4) Angolo relativo phi
    subplot(2,2,4);
    plot(tx, rad2deg(X(4,:)), 'm-', 'LineWidth', 2); hold on;
    yline( 60, '--r', '\phi_{max}', 'LineWidth', 1.2, 'FontSize', 12);
    yline(-60, '--r', '\phi_{min}', 'LineWidth', 1.2, 'FontSize', 12);
    xlabel('Tempo [s]', 'FontSize', 14);
    ylabel('\phi [°]',  'FontSize', 14);
    title('Angolo relativo biciclo-rimorchio \phi', 'FontSize', 15, 'FontWeight', 'bold');
    legend('Reale','\phi_{max}','Location','best','FontSize',12);
    grid on; set(gca,'FontSize',13,'GridAlpha',0.4); hold off;

    sgtitle(['Stati del sistema – ' fig_title], 'FontSize', 17, 'FontWeight', 'bold');

    %% FIGURA 3 — Errore di Tracking
    err = sqrt((X(1,1:Nsim)-x_ref(1,1:Nsim)).^2 + (X(2,1:Nsim)-x_ref(2,1:Nsim)).^2);
    te  = (0:length(err)-1)*dt;

    figure('Color','w','Name',[fig_title ' – Errore']);
    ax = gca;
    fill([te, fliplr(te)], [err, zeros(1,length(err))],[0.68 0.85 0.90], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    hold on;
    plot(te, err, 'b-', 'LineWidth', 2);
    xlabel('Tempo [s]',  'FontSize', 14);
    ylabel('Errore [m]', 'FontSize', 14);
    title('Errore di Tracking', 'FontSize', 16, 'FontWeight', 'bold');
    lg2 = legend('Errore', 'Location', 'best');
    lg2.FontSize = 13;
    grid on;
    ax.FontSize = 13;
    set(ax, 'GridAlpha', 0.4);
    xlim([0, te(end)]);
    ylim([0, max(err)*1.05]);
    hold off;

    %% FIGURA 4 — Ingressi di controllo
    figure('Color','w','Name',[fig_title ' – Ingressi']);
    sgtitle(['Ingressi di controllo – ' fig_title], 'FontSize', 17, 'FontWeight', 'bold');

    subplot(1,2,1);
    stairs(tu, U(1,:), 'b-', 'LineWidth', 2); hold on;
    if any(u_ref(1,:) ~= 0)
        plot(tu, u_ref(1,1:Nsim), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
    end
    yline(0, '--k', 'LineWidth', 1.0);
    xlabel('Tempo [s]',   'FontSize', 14);
    ylabel('v_b [m/s]',   'FontSize', 14);
    title('Velocit\`a v_b', 'FontSize', 15, 'FontWeight', 'bold');
    grid on; set(gca,'FontSize',13,'GridAlpha',0.4); hold off;

    subplot(1,2,2);
    stairs(tu, rad2deg(U(2,:)), 'r-', 'LineWidth', 2); hold on;
    yline( 35, '--b', '\delta_{max}', 'LineWidth', 1.2, 'FontSize', 12);
    yline(-35, '--b', '\delta_{min}', 'LineWidth', 1.2, 'FontSize', 12);
    xlabel('Tempo [s]', 'FontSize', 14);
    ylabel('\delta [°]', 'FontSize', 14);
    title('Sterzo \delta', 'FontSize', 15, 'FontWeight', 'bold');
    grid on; set(gca,'FontSize',13,'GridAlpha',0.4); hold off;

    %% Metriche a console
    fprintf('  Errore posizione – medio: %.3f m  |  finale: %.3f m\n', mean(err), err(end));
    fprintf('  |phi| max: %.1f deg\n', rad2deg(max(abs(X(4,:)))));
end
