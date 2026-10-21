function [O_star, rk_final, k, flag_obs] = observability_codistribution(f, G, h, X, X0, max_iter)
    % versione vecchia
    fields = [f, G];   

    O      = jacobian(h, X);
    rk_old = rank(double(subs(O, X, X0)));
    k      = 0;
    flag_obs = false;

    while k < max_iter
        k     = k + 1;
        O_new = O;

        for i = 1:size(O, 1)
            for j = 1:size(fields, 2)          
                Lf_ij = O(i,:) * fields(:,j)
                O_new = [O_new; jacobian(Lf_ij, X)];
            end
        end

        rk_new = rank(subs(O_new, X, X0));  

        if rk_new == rk_old
            break;
        end

        O      = O_new;
        rk_old = rk_new;
    end

    O_star   = O;           % forma simbolica
    rk_final = rk_old;

    if rk_final == length(X)
        flag_obs = true;
    end
end