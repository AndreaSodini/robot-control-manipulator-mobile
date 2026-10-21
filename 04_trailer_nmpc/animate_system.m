function animate_system(X, U, t, x_ref, p, title_str, gif_name, ~)
    % Calcola il tempo di campionamento reale dal vettore t
    if length(t) > 1
        Ts_real = t(2) - t(1);
    else
        Ts_real = 0.05; 
    end

    Nsim = size(X,2) - 1;
    pad  = 2;
    all_x = [X(1,:), x_ref(1,:)];
    all_y = [X(2,:), x_ref(2,:)];
    xl = [min(all_x)-pad, max(all_x)+pad];
    yl = [min(all_y)-pad, max(all_y)+pad];
    
    bici_len  = p.L;
    bici_wid  = 0.55;
    trail_len = p.M;
    trail_wid = 0.65;

    rpatch = @(xc,yc,ang,len,wid) deal( ...
        xc + [-len/2  len/2  len/2 -len/2]*cos(ang) ...
           - [-wid/2 -wid/2  wid/2  wid/2]*sin(ang), ...
        yc + [-len/2  len/2  len/2 -len/2]*sin(ang) ...
           + [-wid/2 -wid/2  wid/2  wid/2]*cos(ang));

    % Setup figura 
    fig = figure('Color','w','Position',[60 60 1100 680],'Name',title_str);
    set(fig,'Renderer','opengl');
    ax1 = gca; axis equal; grid on; hold on;
    xlim(xl); ylim(yl);
    xlabel('x [m]','FontSize',14); ylabel('y [m]','FontSize',14);
    title(title_str,'FontSize',16,'FontWeight','bold');
    ax1.FontSize = 13; set(ax1,'GridAlpha',0.4);

    % Disegno Traiettoria Statica (Sfondo) 
    is_circle = (norm(x_ref(1:2,1) - x_ref(1:2,end)) < 0.5) && (size(x_ref,2) > 10);
    if is_circle
        cx_ref = mean(x_ref(1,:));
        cy_ref = mean(x_ref(2,:));
        R_ref  = mean(sqrt((x_ref(1,:)-cx_ref).^2 + (x_ref(2,:)-cy_ref).^2));
        th_c   = linspace(0,2*pi,300);
        plot(cx_ref+R_ref*cos(th_c), cy_ref+R_ref*sin(th_c),'r--','LineWidth',1.2,'HandleVisibility','off');
    else
        plot(x_ref(1,:), x_ref(2,:),'r--','LineWidth',1.2,'HandleVisibility','off');
    end

    % Elementi Mobili
    h_ref_point = plot(NaN,NaN,'ro','MarkerSize',10,'LineWidth',2); % Target mobile
    h_trace_t   = plot(NaN,NaN,'b','LineWidth',1.5);
    h_trace_b   = plot(NaN,NaN,'k','LineWidth',1.0);
    h_trailer   = patch([0 0 0 0],[0 0 0 0],'w','FaceColor','none','EdgeColor','r','LineWidth',1.5);
    h_bici      = patch([0 0 0 0],[0 0 0 0],'w','FaceColor','none','EdgeColor','b','LineWidth',1.5);
    h_pt_trailer = plot(NaN,NaN,'ro','MarkerSize',7,'MarkerFaceColor','r');
    h_pt_vehicle = plot(NaN,NaN,'bo','MarkerSize',7,'MarkerFaceColor','b');

    legend([h_ref_point, h_trace_t, h_trace_b, h_trailer, h_bici], ...
        {'Target Mobile (x\_ref)', 'Traccia rimorchio', 'Traccia veicolo', ...
         'Rimorchio', 'Veicolo (bici)'}, ...
        'Location','bestoutside', 'FontSize', 12);

    % Loop Animazione 
    trace_tx = []; trace_ty = [];
    trace_hx = []; trace_hy = [];

    for k = 1:Nsim+1
        % Dati veicolo
        xt = X(1,k); yt = X(2,k); theta_t = X(3,k); phi = X(4,k);
        theta_b = theta_t + phi;
        
        % Dati target (x_ref)
        xr = x_ref(1,k); yr = x_ref(2,k);

        % Calcolo geometrie veicolo
        xh = xt + p.M*cos(theta_t); yh = yt + p.M*sin(theta_t);
        xf = xh + p.L*cos(theta_b); yf = yh + p.L*sin(theta_b);
        xc_t = (xt+xh)/2; yc_t = (yt+yh)/2;
        xc_b = (xh+xf)/2; yc_b = (yh+yf)/2;

        % AGGIORNAMENTO GRAFICO
        set(h_ref_point, 'XData', xr, 'YData', yr); % Muove il target
        
        trace_tx(end+1) = xt; trace_ty(end+1) = yt;
        trace_hx(end+1) = xh; trace_hy(end+1) = yh;
        set(h_trace_t,'XData',trace_tx,'YData',trace_ty);
        set(h_trace_b,'XData',trace_hx,'YData',trace_hy);

        [tx,ty] = rpatch(xc_t,yc_t,theta_t,trail_len,trail_wid);
        set(h_trailer,'XData',tx,'YData',ty);
        [bx,by] = rpatch(xc_b,yc_b,theta_b,bici_len,bici_wid);
        set(h_bici,'XData',bx,'YData',by);
        
        set(h_pt_trailer,'XData',xt,'YData',yt);
        set(h_pt_vehicle,'XData',xh,'YData',yh);

        drawnow limitrate;

        % Salvataggio GIF
        fr = getframe(fig);
        im = frame2im(fr);
        [imind,cm] = rgb2ind(im,256);
        if k == 1
            imwrite(imind,cm,gif_name,'gif','Loopcount',0,'DelayTime',Ts_real);
        else
            imwrite(imind,cm,gif_name,'gif','WriteMode','append','DelayTime',Ts_real);
        end
    end
end