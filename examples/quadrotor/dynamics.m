% quadrotor dynamics
function Xdot = dynamics(t, X, u, param)

e3 = [0, 0, 1]';
m = param.m;
J = param.J;

[~, v, R, W] = split_to_states(X);

f = u(1);
M = u(2:4);

xdot = v;
vdot = param.g * e3 ...
    - f / m * R * e3;
Wdot = J \ (-wedge(W) * J * W + M);
Rdot = R * wedge(W);

Xdot=[xdot; vdot; Wdot; reshape(Rdot,9,1)];
end