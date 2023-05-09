% This script uses CasADi to autogenerate functions for online Jacobian
% evalutaion

import casadi.*
addpath('Common\');

% load constant physical parameters (these parameters come from a parameter generation function/struct)
m = MX.sym('m',1); 
J = MX.sym('J',3,3); 
dt = MX.sym('dt',1);
g = 9.8; % gravitational acceleration
e3 = [0 0 1]';

% casadi-lize all the variables in the computation
%% state
X = MX.sym('X',3+3+3+9);
% state X:
% position: p = X(1:3)
% velocity: v = X(4:6)
% angular velocity: omega = X(7:9)
% rotation matrix (column-vectorized): R = X(10:18);
p = X(1:3);
v = X(4:6);
omega = X(7:9);
R = reshape(X(10:18),3,3);

%% control input
u = MX.sym('u',4); 
thrust = u(1);
moment = u(2:4);

%% controller parameters
k = MX.sym('k',12);
kp = k(1:3);
kv = k(4:6);
kR = k(7:9);
komega = k(10:12);

% load the controller gains into a struct
k_struct = struct;
k_struct.x = kp;
k_struct.v = kv;
k_struct.R = kR;
k_struct.W = komega;

%% desired values
desired = MX.sym('desired',3+3+3+3+3+3+3+3);
desired_p = desired(1:3);
desired_v = desired(4:6);
desired_p_2dot = desired(7:9);
desired_p_3dot = desired(10:12);
desired_p_4dot = desired(13:15);
desired_b1 = desired(16:18);
desired_b1_dot = desired(19:21);
desired_b1_2dot = desired(22:24);

% load the desired values into a struct
desired_struct = struct;
desired_struct.x = desired_p;
desired_struct.v = desired_v;
desired_struct.x_2dot = desired_p_2dot;
desired_struct.x_3dot = desired_p_3dot;
desired_struct.x_4dot = desired_p_4dot;
desired_struct.b1 = desired_b1;
desired_struct.b1_dot = desired_b1_dot;
desired_struct.b1_2dot = desired_b1_2dot;

%% dynamics
dynamics = MX.sym('dynamics_excludingR',3+3+3+9);

dynamics(1:3) = X(1:3) + dt * v; % pdot = v;
dynamics(4:6) = X(4:6) + dt * (g * e3 - thrust / m * R * e3); % vdot = param.g * e3 - f / m * R * e3;
dynamics(7:9) = X(7:9) + dt * inv(J)* (-wedge(omega) * J * omega + moment); % Wdot = J \ (-wedge(W) * J * W + M);
% dynamics(10:18) = reshape(R * expm(dt * wedge(omega)),9,1);% Rdot = R * wedge(W);
% The computation above is not supported in MATLAB + CasADi because the
% usage of expm requires the SLICOT plugin (which is written in FORTRAN).
% The alternative computation below will be used to replace the expm
% operation on a skew symmetric matrix (refer to https://math.stackexchange.com/questions/879351/matrix-exponential-of-a-skew-symmetric-matrix-without-series-expansion)
omega_norm = norm(omega * dt);
expm_omega_dt = eye(3) + sin(omega_norm)/omega_norm * wedge(omega* dt) + (1-cos(omega_norm))/omega_norm^2 * wedge(omega* dt) * wedge(omega* dt);
dynamics(10:18) = reshape(R * expm_omega_dt,9,1);% Rdot = R * wedge(W);

%% control function h (output is the control actions of total thrust f and moment m)
% position control
[f, Rc, omegac, omegac_dot, ~] = position_control(p, v, R, omega, desired_struct, k_struct, m);

% attitude controller
[M, ~,~] = attitude_control(R, omega, Rc, omegac, omegac_dot, k_struct, J);

% h stores the output of the geometric control
h = [f;M];

%% generate jacobians
grad_f_X = jacobian(dynamics,X);
grad_f_u = jacobian(dynamics,u);
grad_h_X = jacobian(h,X);
grad_h_theta = jacobian(h,k);

% function-lize the generated jacobians
grad_f_X_fcn = Function('grad_f_X_fcn',{X,u,dt,m,J},{grad_f_X});
grad_f_u_fcn = Function('grad_f_u_fcn',{X,u,dt,m,J},{grad_f_u});
grad_h_X_fcn = Function('grad_h_X_fcn',{X,desired,k,m,J},{grad_h_X});
grad_h_theta_fcn = Function('grad_h_theta_fcn',{X,desired,k,m,J},{grad_h_theta});

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