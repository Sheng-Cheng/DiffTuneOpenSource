% This script uses CasADi to autogenerate functions for online Jacobian
% evalutaion

clear all
import casadi.*

% load constant physical parameters
m = MX.sym('m',1);
J = MX.sym('J',1);
dt = MX.sym('dt',1);

% casadi-lize all the variables in the computation
X = MX.sym('X',5);
Xref =  MX.sym('Xref',5);

vd_dot = MX.sym('vd_dot',1);
omegad_dot = MX.sym('omegad_dot',1);

% theta is the parameter collection
theta = MX.sym('theta',4);

% control action
u = MX.sym('u',2);

% split into elementwise control parameters
kp = theta(1);
kd = theta(2);
ktheta = theta(3);
komega = theta(4);

%% dynamics (discretized)
dynamics = X + dt * [X(4).*cos(X(3));
                  X(4).*sin(X(3));
                  X(5);
                  u(1)/m; % u(1) is force F
                  u(2)/J]; % u(2) is moment M

%% control function h 
% function h = controller(X,Xref,vd_dot,omegad_dot,m,J,theta)
% tracking controller (use eq. 16 from https://journals.sagepub.com/doi/pdf/10.5772/51323):
% Inputs:
%   X: actual state
%   Xref: desired/reference state
%   vd_dot: desired/reference linear acceleration
%   omegad_dot: desired/reference angular acceleration
%   m: mass of the vehicle
%   J: inertia of the vehicle
%   theta: controller parameters
% Outputs:
%   h: containing force F and moment M
h = controller(X,Xref,vd_dot,omegad_dot,m,J,theta);

%% generate jacobians
grad_f_X = jacobian(dynamics,X);
grad_f_u = jacobian(dynamics,u);
grad_h_X = jacobian(h,X);
grad_h_theta = jacobian(h,theta);

% function-lize the generated jacobians
grad_f_X_fcn = Function('grad_f_X_fcn',{X,u,dt,m,J},{grad_f_X});
grad_f_u_fcn = Function('grad_h_u_fcn',{X,u,dt,m,J},{grad_f_u});

grad_h_X_fcn = Function('grad_h_Xd_fcn',{X,Xref,vd_dot,omegad_dot,m,J,theta},{grad_h_X});
grad_h_theta_fcn = Function('grad_h_theta_fcn',{X,Xref,vd_dot,omegad_dot,m,J,theta},{grad_h_theta});

% generate mex functions
opts = struct('main', true,...
              'mex', true);

mkdir mex
cd('./mex');
grad_f_X_fcn.generate('grad_f_X_fcn.c',opts);
grad_f_u_fcn.generate('grad_f_u_fcn.c',opts);
grad_h_X_fcn.generate('grad_h_X_fcn.c',opts);
grad_h_theta_fcn.generate('grad_h_theta_fcn.c',opts);

mex grad_f_X_fcn.c -largeArrayDims
mex grad_f_u_fcn.c -largeArrayDims
mex grad_h_X_fcn.c -largeArrayDims
mex grad_h_theta_fcn.c -largeArrayDims
cd('..\')