function [dXdphi,dudphi] = sensitivityComputation(sensitivity,X,Xref,vd_dot,omegad_dot,u,m,J,theta,dt)

% evaluate the Jacobians
dfdX = grad_f_X_fcn(X,u,dt,m,J);
dfdX = full(dfdX);

dfdu = grad_f_u_fcn(X,u,dt,m,J);
dfdu = full(dfdu);

dhdX = grad_h_X_fcn(X,Xref,vd_dot,omegad_dot,m,J,theta);
dhdX = full(dhdX);

dhdtheta = grad_h_theta_fcn(X,Xref,vd_dot,omegad_dot,m,J,theta);
dhdtheta = full(dhdtheta);

% assemble the Jacobians to compute the sensitivity
dXdphi = (dfdX + dfdu * dhdX) * sensitivity + dfdu * dhdtheta;
dudphi = dhdX * sensitivity + dhdtheta;

end
