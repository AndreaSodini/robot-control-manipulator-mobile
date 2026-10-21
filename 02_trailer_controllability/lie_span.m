function B = lie_span(Delta, Delta_k, x)
    B = [];
    for i = 1:size(Delta,2)
        for j = 1:size(Delta_k,2)
            L = lie_bracket(Delta(:,i), Delta_k(:,j), x);
            B = [B, L];
        end
    end
end
