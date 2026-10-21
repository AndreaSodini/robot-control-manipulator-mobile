function [X, U] = mpc_simulate(x0, x_ref, u_ref, t, mpc, p)

    nx   = 4;
    nu   = 2;
    Nsim = length(t) - 1;

    X      = zeros(nx, Nsim+1);
    U      = zeros(nu, Nsim);
    X(:,1) = x0;
    u_prev = u_ref(:,1);

    U_sol = repmat(u_ref(:,1), mpc.Nc, 1);

    opts = optimoptions('fmincon', ...
        'Display',              'off',  ...
        'Algorithm',            'sqp',  ...
        'MaxIterations',        500,    ...
        'MaxFunctionEvaluations', 5000, ...
        'OptimalityTolerance',  1e-5,   ...
        'StepTolerance',        1e-6,   ...
        'ConstraintTolerance',  1e-4);

    lb = repmat(mpc.u_min, mpc.Nc, 1);
    ub = repmat(mpc.u_max, mpc.Nc, 1);

    fprintf('  Simulazione %d passi...\n', Nsim);
    tic;

    for k = 1:Nsim
        if mod(k, 50) == 0
            fprintf('  step %4d/%d  (%.1f s)\n', k, Nsim, toc);
        end

        xk     = X(:, k);
        ref_k  = ref_window(x_ref,  k, mpc.N);
        uref_k = ref_window(u_ref,  k, mpc.N);

        % shift + copia ultimo
        U_sol = [U_sol(nu+1:end); U_sol(end-nu+1:end)];
        % Aggiorna coda con feedforward corrente
        U_sol(end-nu+1:end) = uref_k(:, end);

        cost_fn = @(Uf) mpc_cost(Uf, xk, ref_k, uref_k, mpc, u_prev, p);
        con_fn  = @(Uf) mpc_con(Uf, xk, mpc, p);

        try
            [Uopt, ~, flag] = fmincon(cost_fn, U_sol, [], [], [], [], ...
                                      lb, ub, con_fn, opts);
            if flag <= 0
                Uopt = U_sol;   
            end
        catch
            Uopt = U_sol;
        end

        uk       = clip(Uopt(1:nu), mpc.u_min, mpc.u_max);
        U(:, k)  = uk;
        X(:,k+1) = bicycle_trailer_dynamics(xk, uk, p);
        u_prev   = uk;
        U_sol    = Uopt;
    end

    fprintf('  Completato in %.2f s\n', toc);
end

%%  FUNZIONE DI COSTO
function J = mpc_cost(Uf, x0, ref, u_ref, mpc, u_prev, p)
    nu   = 2;
    U    = reshape(Uf, nu, mpc.Nc);
    x    = x0;
    J    = 0;
    ukm1 = u_prev;

    for i = 1:mpc.N
        uk = U(:, min(i, mpc.Nc));
        x  = bicycle_trailer_dynamics(x, uk, p);

        r    = ref(:,   min(i+1, size(ref,  2)));
        ur   = u_ref(:, min(i,   size(u_ref,2)));

        e    = x - r;
        e(3) = atan2(sin(e(3)), cos(e(3)));   
        e(4) = atan2(sin(e(4)), cos(e(4)));   

        eu = uk - ur;
        du = uk - ukm1;

        if i < mpc.N
            J = J + e'*mpc.Q*e + eu'*mpc.R*eu + du'*mpc.R_du*du;
        else
            J = J + e'*mpc.Qf*e;
        end

        ukm1 = uk;
    end
end

%% VINCOLI HARD
function [c, ceq] = mpc_con(Uf, x0, mpc, p)
    nu  = 2;
    U   = reshape(Uf, nu, mpc.Nc);
    c   = zeros(2*mpc.N, 1);
    ceq = [];
    x   = x0;

    for i = 1:mpc.N
        uk = U(:, min(i, mpc.Nc));
        x  = bicycle_trailer_dynamics(x, uk, p);
        c(2*i-1) =  x(4) - p.phi_max;
        c(2*i)   = -x(4) - p.phi_max;
    end
end

%%  UTILITA'
function rw = ref_window(M, k, N)
    Nc = size(M, 2);
    idx = k : min(k+N, Nc);
    rw  = M(:, idx);
    while size(rw,2) < N+1
        rw = [rw, rw(:,end)];
    end
end

function y = clip(x, lo, hi)
    y = max(lo, min(hi, x));
end
