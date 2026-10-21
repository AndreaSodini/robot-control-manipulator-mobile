function L = lie_bracket(f, g, x)
    Jf = jacobian(f, x);
    Jg = jacobian(g, x);
    L  = Jg*f - Jf*g;
end
