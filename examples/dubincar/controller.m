% tracking controller (use eq. 16 from https://journals.sagepub.com/doi/pdf/10.5772/51323):
% Inputs:
%   X: actual state
%   Xref: desired/reference state
%   vd_dot: desired/reference linear acceleration
%   omegad_dot: desired/reference angular acceleration
%   m: mass of the vehicle
%   J: inertia of the vehicle
%   k_vec: controller parameters
% Outputs:
%   ud: containing force F and moment M

function ud = controller(X,Xref,vd_dot,omegad_dot,m,J,k_vec)
kp = k_vec(1); 
kd = k_vec(2); 
ktheta = k_vec(3); 
komega = k_vec(4); 

% PD controller
F = m * kp* dot([Xref(1) - X(1),Xref(2) - X(2)],[cos(X(3)),sin(X(3))]) ...
    + m * kd * dot([Xref(4)*cos(Xref(3)) - X(4)*cos(X(3)),Xref(4)*sin(Xref(3)) - X(4)*sin(X(3))],[cos(X(3)),sin(X(3))]) ...
    + m * dot([vd_dot*cos(Xref(3)) - Xref(4)*Xref(5)*sin(Xref(3)) vd_dot*sin(Xref(3)) + Xref(4)*Xref(5)*cos(Xref(3))],[cos(X(3)),sin(X(3))]);
M = J * ktheta * (Xref(3) - X(3)) + J * komega * (Xref(5) - X(5)) + J * omegad_dot;

ud = [F;M];