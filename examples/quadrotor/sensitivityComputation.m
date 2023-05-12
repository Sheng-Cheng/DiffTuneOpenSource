% This function computes the sensitivity of quadrotor dynamics wrt geometric
% controller parameters using CasAdi's precomputed Jacobians
function [dxdtheta_next, dhdtheta_current] = sensitivityComputation(dxdtheta_current, X, u, desired, param, k)
% extend desired and parameters to vectors
desired_vec = [desired.x;
    desired.v;
    desired.x_2dot;
    desired.x_3dot;
    desired.x_4dot;
    desired.b1;
    desired.b1_dot;
    desired.b1_2dot];
k_vec = [k.x;
    k.v;
    k.R;
    k.W];

% evaluate the Jacobians
dfdX = grad_f_X_fcn(X,u,param.dt,param.m,param.J);
dfdX = full(dfdX);

dfdu = grad_f_u_fcn(X,u,param.dt,param.m,param.J);
dfdu = full(dfdu);

dhdX = grad_h_X_fcn(X,desired_vec,k_vec,param.m,param.J);
dhdX = full(dhdX);

dhdtheta = grad_h_theta_fcn(X,desired_vec,k_vec,param.m,param.J);
dhdtheta = full(dhdtheta);

% assemble the Jacobians to compute the sensitivity
dxdtheta_next = (dfdX + dfdu * dhdX) * dxdtheta_current + dfdu * dhdtheta;
dhdtheta_current = dhdX * dxdtheta_current + dhdtheta;

end


