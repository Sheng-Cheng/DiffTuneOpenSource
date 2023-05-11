function dXdt = dynamics(t,X,u,param)
dXdt = zeros(5,1);

m = param(1); % mass
J = param(2); % inertia

dXdt(1) = X(4).*cos(X(3));
dXdt(2) = X(4).*sin(X(3));
dXdt(3) = X(5);
dXdt(4) = u(1)./m; % u(1) is force F
dXdt(5) = u(2)./J; % u(2) is moment M
end