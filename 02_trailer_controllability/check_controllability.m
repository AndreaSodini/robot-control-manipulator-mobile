function [isControllable, report] = check_controllability(f, g, x, x0)
    syms d0 d1

    n = length(x);
    m = length(g);
    tol = 1e-6;

    report = struct();
    isControllable = false;

    flag_fx0 = false;
    fx0 = subs(f, [x; d0; d1], [x0; 1.5; 1]);
    if fx0 == zeros(7,1)
        flag_fx0 = true;
    end

    %% (i) f(x) = 0 in Be(x0)
    eps = 1e-2;
    Ns = 50;
    ok = true;

    for k = 1:Ns
        dx = eps * (2*rand(length(x),1)-1);
        xk = x0 + dx;
        fk = subs(f, x, xk);
        if norm(fk) > tol
            ok = false;
            break
        end
    end

    report.cond_i = ok;

    if report.cond_i
        isControllable = true;
        return
    end

    %% (ii) f(x) ∈ span{g_i(x)} in Be(x0)
    Gx0 = subs(g, x, x0);
    fx0 = subs(f, x, x0);

    rank_G = rank(Gx0);
    rank_Gf = rank([Gx0 fx0]);

    report.cond_ii = (rank_G == rank_Gf);

    if report.cond_ii
        isControllable = true;
        return
    end

    %% (iii) dim Δ_L(x0) = n
    Delta0 = g;
    Delta = [f, g];

    [Delta_L, ~, ~, flag] = delta_invariant_closure(Delta0, Delta, x, x0);

    report.cond_iii = flag;

    if report.cond_iii && flag_fx0
        isControllable = true;
        return
    end


    %% (iv) dim (Δ_L + Δ_G1)(x0) = n
    Delta_G1 = [];
    for i = 1:m
        for j = 1:m
            Delta_G1 = [Delta_G1 lie_bracket(g(:,i), g(:,j), x)];
        end
    end

    Delta_sum = [Delta_L Delta_G1];
    Delta_sum_x0 = subs(Delta_sum, x, x0);
    report.cond_iv = (rank(Delta_sum_x0) == n);

    if report.cond_iv && flag_fx0
        isControllable = true;
        return
    end
end



