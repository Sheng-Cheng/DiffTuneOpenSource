% This script uses CasADi to autogenerate functions for online Jacobian
% evalutaion

clear all
import casadi.*

%% define the dimensions
% dim_state = 5; % dimension of system state
% dim_control = 2;  % dimension of control inputs
% dim_controllerParameters = 4;  % dimension of controller parameters

%% load constant physical parameters, e.g., mass or sample time
% m = MX.sym('m',1);  % mass
% dt = MX.sym('dt',1); % sample time

%% casadi-lize all the variables in the computation, e.g., system state
% X = MX.sym('X',dim_state);  % system state

%% theta is the collection of controller parameters 
% theta = MX.sym('theta',dim_controllerParameters);

%% define the control input
% u = MX.sym('u',dim_control);

%% define the dynamics (discretization via Forward Euler or Runge Kutta)
% dynamics = X + dt * ...

%% compute the control action, denoted by h
% h = controller(...);

%% generate jacobians
% grad_f_X = jacobian(dynamics,X);
% grad_f_u = jacobian(dynamics,u);
% grad_h_X = jacobian(h,X);
% grad_h_theta = jacobian(h,theta);

%% function-lize the generated jacobians
% inputs_f and inputs_h denote the input arguments to the dynamics and controller h, respectively
% grad_f_X_fcn = Function('grad_f_X_fcn',{inputs_f},{grad_f_X});
% grad_f_u_fcn = Function('grad_h_u_fcn',{inputs_f},{grad_f_u});
% grad_h_X_fcn = Function('grad_h_Xd_fcn',{inputs_h},{grad_h_X});
% grad_h_theta_fcn = Function('grad_h_theta_fcn',{inputs_h},{grad_h_theta});

%% generate mex functions
% opts = struct('main', true,...
%               'mex', true);
% 
% mkdir mex
% cd('./mex');
% grad_f_X_fcn.generate('grad_f_X_fcn.c',opts);
% grad_f_u_fcn.generate('grad_f_u_fcn.c',opts);
% grad_h_X_fcn.generate('grad_h_X_fcn.c',opts);
% grad_h_theta_fcn.generate('grad_h_theta_fcn.c',opts);

% mex grad_f_X_fcn.c -largeArrayDims
% mex grad_f_u_fcn.c -largeArrayDims
% mex grad_h_X_fcn.c -largeArrayDims
% mex grad_h_theta_fcn.c -largeArrayDims
% cd('..\')