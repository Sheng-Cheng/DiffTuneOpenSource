function [f, Rc, Wc, Wc_dot, error] = ...
    position_control(x, v, R, W, desired, k, m)

e3 = [0, 0, 1]';
g = 9.8;

error.x = x - desired.x;
error.v = v - desired.v;
A = - k.x .* error.x ...
    - k.v .* error.v ...
    - m * g * e3 ...
    + m * desired.x_2dot;

%%
b3 = R * e3;
f = -dot(A, b3);
ev_dot = g * e3 ...
    - f / m * b3 ...
    - desired.x_2dot;
A_dot = - k.x .* error.v ...
    - k.v .* ev_dot ...
    + m * desired.x_3dot;
%%
b3_dot = R * wedge(W) * e3;
f_dot = -dot(A_dot, b3) - dot(A, b3_dot);
ev_2dot = - f_dot / m * b3 - f / m * b3_dot - desired.x_3dot;
A_ddot = - k.x .* ev_dot ...
    - k.v .* ev_2dot ...
    + m * desired.x_4dot;
%%
[b3c, b3c_dot, b3c_ddot] = deriv_unit_vector(-A, -A_dot, -A_ddot);

A2 = -wedge(desired.b1) * b3c;
A2_dot = -wedge(desired.b1_dot) * b3c - wedge(desired.b1) * b3c_dot;
A2_ddot = - wedge(desired.b1_2dot) * b3c ...
    - 2 * wedge(desired.b1_dot) * b3c_dot ...
    - wedge(desired.b1) * b3c_ddot;

[b2c, b2c_dot, b2c_ddot] = deriv_unit_vector(A2, A2_dot, A2_ddot);

b1c = wedge(b2c) * b3c;
b1c_dot = wedge(b2c_dot) * b3c + wedge(b2c)*b3c_dot;
b1c_ddot = wedge(b2c_ddot) * b3c ...
    + 2 * wedge(b2c_dot) * b3c_dot ...
    + wedge(b2c) * b3c_ddot; 
%%
Rc = [b1c, b2c, b3c];
Rc_dot = [b1c_dot, b2c_dot, b3c_dot];
Rc_ddot = [b1c_ddot, b2c_ddot, b3c_ddot];
%%
Wc = vee(Rc' * Rc_dot);
Wc_dot = vee(Rc' * Rc_ddot - wedge(Wc)^2);
end