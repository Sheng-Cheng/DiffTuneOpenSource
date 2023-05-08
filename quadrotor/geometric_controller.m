function [f, M, error] = geometric_controller(X, desired, k, param)
% split states
[x, v, R, W] = split_to_states(X);

% Run position controller
[f, Rc, Wc, Wc_dot, error] = position_control(x, v, R, W, desired, k, param.m);

% Run attitude controller
[M, error.R, error.W] = attitude_control(R, W, Rc, Wc, Wc_dot, k, param.J);

end