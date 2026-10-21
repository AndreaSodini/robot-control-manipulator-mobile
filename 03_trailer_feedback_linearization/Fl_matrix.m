clear all; clc; close all;

syms v M L delta theta_t x1 y1 phi v_t d u1 u2 omega real

%% Sistema Biciclo con Rimorchio
x1_dot      = v_t * cos(theta_t);
y1_dot      = v_t * sin(theta_t);
theta_t_dot = omega;                            % omega è la variabile ausiliaria = v_t*tan(phi)/M = theta_t_dot                     
phi_dot     = v * tan(delta) / L - omega;

y_out = [x1 + d*cos(theta_t);
         y1 + d*sin(theta_t)];

X  = [x1; y1; theta_t; phi];
dX = [x1_dot; y1_dot; theta_t_dot; phi_dot];

%% Derivata dell'uscita y_out
dy = jacobian(y_out, X) * dX;
dy = simplify(dy);
disp('dy ='); disp(dy)

%% Sostituzione ingressi virtuali: u1=v_t, u2=omega
dy_u = subs(dy, [v_t, omega], [u1, u2]);
disp('dy_u ='); disp(dy_u)

%% Estrazione A e B
b0  = subs(dy_u, {u1,u2}, {0,0});

A11 = subs(dy_u(1), {u1,u2}, {1,0}) - b0(1);
A12 = subs(dy_u(1), {u1,u2}, {0,1}) - b0(1);
A21 = subs(dy_u(2), {u1,u2}, {1,0}) - b0(2);
A22 = subs(dy_u(2), {u1,u2}, {0,1}) - b0(2);

A = simplify([A11 A12; A21 A22]);
B = simplify(b0);

disp('A (decoupling matrix) =');  disp(A)
disp('B (drift) =');              disp(B)
disp('rank(A) =');                disp(rank(A))
