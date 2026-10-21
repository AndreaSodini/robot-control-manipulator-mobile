clear all; close all; clc;

% Sistema Non Lineare Biciclo con Rimorchio
syms v d0 d1 delta theta0 theta1 x1 y1 omega acc acc_dot t r w

x1_dot     = v*cos(theta0)-d1*cos(theta1);   
y1_dot     = v*sin(theta0)-d1*sin(theta1);    
delta_dot  = omega;
v_dot      = acc;
acc_d      = acc_dot;
theta0_dot = (v/d0)*tan(delta);
theta1_dot = (v/d1)*sin(theta0 - theta1);

y = [x1;y1];

% Ritardo 1 passo ingresso acc
X = [x1; y1; delta; v; theta0; theta1; acc];
X0 = [0;0;0;1;0;0;0];
%X0 = ones(7,1);
U = [acc_dot; omega];
dX = [x1_dot; y1_dot; delta_dot; v_dot; theta0_dot; theta1_dot; acc_d];

F = subs(dX, U, [0; 0]);
dXr = dX - F;
G = jacobian(dXr, U);
Ga = G(1);
Go = G(2);

% STLA & Controllabilità
delta0 = G;
delta = [F,G];

Delta_0_eval = subs(delta0, X, X0);
rk_delta0 = rank(Delta_0_eval);

disp("--- SHORT TIME LOCAL ACCESSIBILITY ---");
if (rk_delta0 ~= size(X,1))
    [Delta_star, rk_final, k, flag_stla] = delta_invariant_closure(delta0, delta, X, X0);
    disp("Final Distribution Rank: " + rk_final);
    disp("Number of Iterations: " + k);
    if(flag_stla)
        disp("Invariant Closure reached STLA");
        disp(newline + "--- SHORT TIME LOCAL CONTROLLABILITY ---");
        [isControllable, report] = check_controllability(F, G, X, X0);
        if(isControllable)
           disp("System Locally Controllable");
        else
           disp("System Locally NOT Controllable");
        end 
    end
else
    disp("Rank of Delta0 is equal to size of State -> STLA");
end

% WLA
if(~flag_stla)
    disp(newline + "--- WEAK LOCAL ACCESSIBILITY ---");
    [Delta_star, rk_final, k, flag_wla] = delta_invariant_closure(delta, delta, X, X0);
    disp("Final Distribution Rank: " + rk_final);
    disp("Number of Iterations: " + k);
    if(flag_wla)
        disp("Invariant Closure reached WLA");
    end
end

% Osservabilità
max_iter = 15;
[O_star, rk_final, k, flag_obs] = observability_codistribution(F, G, y, X, X0, max_iter);

disp(newline + "--- OBSERVABILITY ---");
disp("Final Distribution Rank: " + rk_final);
disp("Number of Iterations: " + k);

if flag_obs
    disp("Observability Codistribution is full rank -> OBSERVABLE");
else
    disp("System NOT observable");
    % spazio di inosservabilità
    O_num = double(subs(O_star, X, X0));
    dO_ort = null(O_num);
    disp("Inobservability Space:"); disp(dO_ort);
end