close all;
fprintf('Avvio animazione \n');
skip = 5;
idx  = 1:skip:length(T_sim);

%% Setup
bici_len  = 1.5;
bici_wid  = 0.55;
trail_len = 1;
trail_wid = 0.65;
pad = 2;
xl  = [min(x_t)-pad, max(x_t)+pad];
yl  = [min(y_t)-pad, max(y_t)+pad];

figure('Color','w');
set(gcf,'Renderer','opengl')
set(gcf, 'Position', [100, 100, 1074, 646]);

ax1 = gca;
axis equal; grid on; hold on;
xlim(xl); ylim(yl);
xlabel('x [m]', 'FontSize', 14);
ylabel('y [m]', 'FontSize', 14);
title('Trailer Control - Inseguimento Traiettoria', 'FontSize', 16, 'FontWeight', 'bold');

% --- Riferimento fisso (Cerchio) ---
th = linspace(0,2*pi,300);
plot(cx + R_ref*cos(th), cy + R_ref*sin(th), 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');

% --- Target ---
h_target = plot(NaN, NaN, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Target (xr\_ref, yr\_ref)');

% --- Tracce veicoli ---
h_trace_t = plot(NaN,NaN,'b','LineWidth',1.2, 'DisplayName', 'Traccia Trailer');
h_trace_b = plot(NaN,NaN,'k:','LineWidth',1, 'DisplayName', 'Traccia Bici');

% --- Figure Veicoli ---
h_trailer = patch([0 0 0 0],[0 0 0 0], 'w','FaceColor','none','EdgeColor','r','LineWidth',1.5, 'DisplayName', 'Rimorchio');
h_bici    = patch([0 0 0 0],[0 0 0 0], 'w','FaceColor','none','EdgeColor','b','LineWidth',1.5, 'DisplayName', 'Bici');

% --- Assi Posteriori ---
h_pt_trailer = plot(NaN,NaN,'ro','MarkerSize',7,'MarkerFaceColor','r', 'DisplayName', 'Asse post. Rimorchio');
h_pt_vehicle = plot(NaN,NaN,'bo','MarkerSize',7,'MarkerFaceColor','b', 'DisplayName', 'Asse post. Bici');

rpatch = @(xc,yc,ang,len,wid) deal( ...
    xc + [-len/2 len/2 len/2 -len/2]*cos(ang) - [-wid/2 -wid/2 wid/2 wid/2]*sin(ang), ...
    yc + [-len/2 len/2 len/2 -len/2]*sin(ang) + [-wid/2 -wid/2 wid/2 wid/2]*cos(ang));

legend('Location','bestoutside','FontSize',10);

v = VideoWriter('trailer_tracking_full.mp4','MPEG-4');
v.FrameRate = 30;
open(v)

trace_tx=[]; trace_ty=[];
trace_hx=[]; trace_hy=[];

%% Loop di Animazione
for k = idx
    xt = x_t(k);  yt = y_t(k);
    xh = x_h(k);  yh = y_h(k);
    xf = x_f(k);  yf = y_f(k);
    tt = theta_t(k);
    tb = theta_b(k);
    
    % Aggiornamento Target e Punto d
    set(h_target, 'XData', xr_ref(k), 'YData', yr_ref(k));
    xd = xt + d*cos(tt);
    yd = yt + d*sin(tt);
    
    % Centri veicoli
    xc_t = (xt + xh)/2; yc_t = (yt + yh)/2;
    xc_b = (xh + xf)/2; yc_b = (yh + yf)/2;
    
    % Aggiornamento Tracce
    trace_tx(end+1)=xt; trace_ty(end+1)=yt;
    trace_hx(end+1)=xh; trace_hy(end+1)=yh;
    set(h_trace_t,'XData',trace_tx,'YData',trace_ty);
    set(h_trace_b,'XData',trace_hx,'YData',trace_hy);
    
    % Aggiornamento Figure
    [tx,ty] = rpatch(xc_t,yc_t,tt,trail_len,trail_wid);
    set(h_trailer,'XData',tx,'YData',ty);
    [bx,by] = rpatch(xc_b,yc_b,tb,bici_len,bici_wid);
    set(h_bici,'XData',bx,'YData',by);
    
    % Aggiornamento Assi Posteriori
    set(h_pt_trailer, 'XData', xt, 'YData', yt);
    set(h_pt_vehicle, 'XData', xh, 'YData', yh);
    
    drawnow;
    writeVideo(v, getframe(gcf));
end

close(v)
fprintf('Animazione completata.\n');