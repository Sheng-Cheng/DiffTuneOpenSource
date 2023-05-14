% This function defines how the sensitivity propgation runs at each sample time
% The inputs_f and inputs_h are of the same form to those used in templateAutoGeneration.m

% function [dXdphi,dudphi] = sensitivityComputation(sensitivity,...)

%% evaluate the Jacobians
% dfdX = grad_f_X_fcn(inputs_f);
% dfdX = full(dfdX);
% 
% dfdu = grad_f_u_fcn(inputs_f);
% dfdu = full(dfdu);
% 
% dhdX = grad_h_X_fcn(inputs_h);
% dhdX = full(dhdX);
% 
% dhdtheta = grad_h_theta_fcn(inputs_h);
% dhdtheta = full(dhdtheta);

%% assemble the Jacobians to compute the sensitivity
% dXdphi = (dfdX + dfdu * dhdX) * sensitivity + dfdu * dhdtheta;
% dudphi = dhdX * sensitivity + dhdtheta;

% end
