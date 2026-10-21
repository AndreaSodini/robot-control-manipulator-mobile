function x_next = bicycle_trailer_dynamics(x, u, p)
    dt = p.dt;
    k1 = f_ode(x,           u, p);
    k2 = f_ode(x + dt/2*k1, u, p);
    k3 = f_ode(x + dt/2*k2, u, p);
    k4 = f_ode(x + dt*k3,   u, p);
    x_next    = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x_next(3) = atan2(sin(x_next(3)), cos(x_next(3)));
    x_next(4) = atan2(sin(x_next(4)), cos(x_next(4)));
end

function dx = f_ode(x, u, p)
    theta_t = x(3);
    phi     = x(4);
    v_b     = u(1);
    delta   = u(2);

    phi   = max(-p.phi_max,   min(p.phi_max,   phi));
    delta = max(-p.delta_max, min(p.delta_max, delta));

    theta_dot = v_b * sin(phi) / p.M;

    dx = zeros(4,1);
    dx(1) = v_b * cos(phi) * cos(theta_t);        % x_t_dot
    dx(2) = v_b * cos(phi) * sin(theta_t);        % y_t_dot
    dx(3) = theta_dot;                            % theta_t_dot
    dx(4) = v_b * tan(delta) / p.L - theta_dot;   % phi_dot
end
