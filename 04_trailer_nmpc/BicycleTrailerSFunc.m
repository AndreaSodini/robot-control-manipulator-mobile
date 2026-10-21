function BicycleTrailerSFunc(block)
    block.NumInputPorts  = 1;    % u = [v_b; delta]
    block.NumOutputPorts = 1;    % x = [x_r; y_r; theta_r; phi]
    block.NumContStates  = 4;

    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    block.InputPort(1).Dimensions        = 2;
    block.InputPort(1).DirectFeedthrough = false;
    block.OutputPort(1).Dimensions       = 4;

    % Parametri: [L, M, phi_max, delta_max]
    block.NumDialogPrms  = 4;
    block.SampleTimes    = [0 0];   % tempo continuo
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('InitializeConditions', @IC);
    block.RegBlockMethod('Outputs',              @Output);
    block.RegBlockMethod('Derivatives',          @Deriv);
end

function IC(block)
    % Stato iniziale: rimorchio sul cerchio, heading verso l'alto
    block.ContStates.Data = [3; 0; pi/2; 0];
end

function Output(block)
    block.OutputPort(1).Data = block.ContStates.Data;
end

function Deriv(block)
    x     = block.ContStates.Data;
    u     = block.InputPort(1).Data;

    L         = block.DialogPrm(1).Data;   % interasse biciclo
    M         = block.DialogPrm(2).Data;   % lunghezza gancio
    phi_max   = block.DialogPrm(3).Data;   % max angolo relativo
    delta_max = block.DialogPrm(4).Data;   % max sterzo

    theta_r = x(3);
    phi     = x(4);
    v_b     = u(1);
    delta   = u(2);

    % Saturazione
    phi   = max(-phi_max,   min(phi_max,   phi));
    delta = max(-delta_max, min(delta_max, delta));
    v_b   = max(-0.2,       min(1.0,       v_b));

    theta_r_dot = v_b * sin(phi) / M;

    dx    = zeros(4,1);
    dx(1) = v_b * cos(phi) * cos(theta_r);        % x_r_dot
    dx(2) = v_b * cos(phi) * sin(theta_r);        % y_r_dot
    dx(3) = theta_r_dot;                           % theta_r_dot
    dx(4) = v_b * tan(delta) / L - theta_r_dot;   % phi_dot

    block.Derivatives.Data = dx;
end
