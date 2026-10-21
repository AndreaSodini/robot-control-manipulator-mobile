function [Delta_star, rk_next, k, flag] = delta_invariant_closure(Delta0, Delta, x, xbar)
    Delta_k = Delta0;
    n = length(x);
    k = 0;
    flag = false;

    % rango iniziale
    rk_prev = -1;

    while true
        k = k + 1;
        B = lie_span(Delta, Delta_k, x);

        Delta_next = [Delta_k, B];

        Delta_next_eval = subs(Delta_next, x, xbar);
        rk_next = rank(Delta_next_eval);

        % criterio di arresto
        if rk_next == rk_prev
            Delta_k = Delta_next;
            rk_prev = rk_next;
            break;
        end

        if rk_next >= n
            flag = true;
            Delta_k = Delta_next;
            rk_prev = rk_next;
            break;
        end

        % aggiorna
        Delta_k = Delta_next;
        rk_prev = rk_next;
        
    end

    Delta_star = Delta_k;
end

