%% Parametri generali 
GIF_IDEALE = 'robot_CT_ideale.gif';
GIF_REALE  = 'robot_CT_reale.gif';

N_FRAMES   = 120;      % numero di frame da campionare
FRAME_DT   = 0.08;     % [s] delay tra frame nella GIF 
AZ = 135;   
EL = 18;   

% Colori tracce
COL_REAL   = [0.8 0.2 0.2];    
COL_TRAJ   = [0.1 0.7 0.3];    
% Colori link del robot
COL_LINK   = [0.85 0.90 0.95]; 
COL_JOINT  = [0.75 0.80 0.88]; 
ALPHA_ROB  = 0.92;             

%% Dati  
q_data      = out.q.Data;        % configurazioni ideali
q_real_data = out.q_real.Data;   % configurazioni reali
t           = out.q.Time;

idx = round(linspace(1, size(q_data,1), N_FRAMES));

%% Traiettoria EE 
ee_name = robot.Bodies{end}.Name;

p_trace      = zeros(length(idx), 3);
p_trace_real = zeros(length(idx), 3);

for f = 1:length(idx)
    ii = idx(f);
    T1 = getTransform(robot,      q_data(ii,:)',      ee_name);
    T2 = getTransform(robot_real, q_real_data(ii,:)', ee_r);
    p_trace(f,:)      = T1(1:3,4)';
    p_trace_real(f,:) = T2(1:3,4)';
end

%%  GIF 1 — Robot ideale (CT)
fprintf('Generazione GIF 1: %s ...\n', GIF_IDEALE);

fig1 = figure('Name','CT Ideale','Color','white','Position',[100 100 800 600]);
ax1  = axes('Parent', fig1);   % asse creato UNA VOLTA SOLA

for f = 1:N_FRAMES
    ii = idx(f);
    cla(ax1, 'reset');

    % Disegna robot
    show(robot, q_data(ii,:)','Parent', ax1,'PreservePlot', false,'Frames', 'off','Visuals', 'on');

    hold(ax1, 'on');
    lightenRobot(ax1, COL_LINK, COL_JOINT, ALPHA_ROB);

    % Traccia EE 
    if f > 1
        plot3(ax1, p_trace(1:f,1), p_trace(1:f,2), p_trace(1:f,3),'-', 'Color', COL_TRAJ, 'LineWidth', 2);
    end
    plot3(ax1, p_trace(f,1), p_trace(f,2), p_trace(f,3),'o', 'Color', COL_TRAJ, 'MarkerFaceColor', COL_TRAJ, 'MarkerSize', 6);

    % Estetica
    view(ax1, AZ, EL);
    grid(ax1, 'on');
    axis(ax1, 'equal');
    xlabel(ax1, 'X [m]', 'FontSize', 11);
    ylabel(ax1, 'Y [m]', 'FontSize', 11);
    zlabel(ax1, 'Z [m]', 'FontSize', 11);
    title(ax1, sprintf('CT — Modello Ideale   t = %.2f s', t(ii)),'FontSize', 13, 'FontWeight', 'bold');
    xlim(ax1, [-0.6  0.6]);
    ylim(ax1, [-0.6  0.6]);
    zlim(ax1, [ 0.0  0.8]);

    drawnow;
    figure(fig1);   

    % Salva GIF
    frame = getframe(fig1);
    im    = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);

    if f == 1
        imwrite(imind, cm, GIF_IDEALE, 'gif','Loopcount', Inf, 'DelayTime', FRAME_DT);
    else
        imwrite(imind, cm, GIF_IDEALE, 'gif','WriteMode', 'append', 'DelayTime', FRAME_DT);
    end

    if mod(f, 20) == 0
        fprintf('  frame %d/%d\n', f, N_FRAMES);
    end
end

close(fig1);

%%  GIF 2 — Robot reale (CT con incertezze)
fprintf('Generazione GIF 2: %s ...\n', GIF_REALE);

fig2 = figure('Name','CT Adattivo','Color','white','Position',[100 100 800 600]);
ax2  = axes('Parent', fig2);   % asse creato UNA VOLTA SOLA

for f = 1:N_FRAMES
    ii = idx(f);

    cla(ax2, 'reset');

    % Disegna robot reale
    show(robot_real, q_real_data(ii,:)','Parent', ax2,'PreservePlot', false,'Frames', 'off','Visuals', 'on');

    hold(ax2, 'on');
    lightenRobot(ax2, COL_LINK, COL_JOINT, ALPHA_ROB);

    % Traccia EE reale
    if f > 1
        plot3(ax2, p_trace_real(1:f,1), p_trace_real(1:f,2), p_trace_real(1:f,3), ...
              '-', 'Color', COL_REAL, 'LineWidth', 2);
    end
    plot3(ax2, p_trace_real(f,1), p_trace_real(f,2), p_trace_real(f,3), ...
          'o', 'Color', COL_REAL, 'MarkerFaceColor', COL_REAL, 'MarkerSize', 6);

    % Estetica
    view(ax2, AZ, EL);
    grid(ax2, 'on');
    axis(ax2, 'equal');
    xlabel(ax2, 'X [m]', 'FontSize', 11);
    ylabel(ax2, 'Y [m]', 'FontSize', 11);
    zlabel(ax2, 'Z [m]', 'FontSize', 11);
    title(ax2, sprintf('CT ADATTIVO— Modello Reale (incertezze)   t = %.2f s', t(ii)), ...
          'FontSize', 13, 'FontWeight', 'bold');
    xlim(ax2, [-0.6  0.6]);
    ylim(ax2, [-0.6  0.6]);
    zlim(ax2, [ 0.0  0.8]);

    legend(ax2, '', 'Traj. ideale (rif.)', 'Traj. reale', ...
           'Location', 'northeast', 'FontSize', 10);

    drawnow;
    figure(fig2);   

    % Salva GIF
    frame = getframe(fig2);
    im    = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);

    if f == 1
        imwrite(imind, cm, GIF_REALE, 'gif', ...
                'Loopcount', Inf, 'DelayTime', FRAME_DT);
    else
        imwrite(imind, cm, GIF_REALE, 'gif', ...
                'WriteMode', 'append', 'DelayTime', FRAME_DT);
    end

    if mod(f, 20) == 0
        fprintf('  frame %d/%d\n', f, N_FRAMES);
    end
end


function lightenRobot(ax, col_link, col_joint, alpha_val)
    patches = findobj(ax, 'Type', 'Patch');
    for k = 1:length(patches)
        patches(k).FaceColor = col_link;
        patches(k).EdgeColor = 'none';
        patches(k).FaceAlpha = alpha_val;
    end
    surfs = findobj(ax, 'Type', 'Surface');
    for k = 1:length(surfs)
        surfs(k).FaceColor = col_joint;
        surfs(k).EdgeColor = 'none';
        surfs(k).FaceAlpha = alpha_val;
    end
end
