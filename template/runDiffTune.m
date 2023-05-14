% Use this script to run the simulation with DiffTune

close all;
clear all;

addpath('mex\');
import casadi.*

%% define the dimensions
% dim_state = 5; % dimension of system state
% dim_control = 2;  % dimension of control inputs
% dim_controllerParameters = 4;  % dimension of controller parameters

%% Define simulation parameters (e.g., sample time dt, duration, etc)
% dt = 0.01;
% time = 0:dt:10;

%% Initialize controller gains (must be a vector of size dim_controllerParameters x 1)
% theta = ...; 

%% Define desired trajectory if necessary

%% Initialize variables for DiffTune iterations
% learningRate = 2;  
% maxIterations = 100;
% itr = 0;

% loss_hist = [];  % storage of the loss value in each iteration
% param_hist = []; % storage of the parameter value in each iteration
% gradientUpdate = zeros(dim_controllerParameters,1); % define the parameter update at each iteration

%% DiffTune iterations
% while (1)
    % itr = itr + 1;

    % initialize state
    % X_storage = zeros(dim_state,1);
    
    % initialize sensitivity
    % dx_dtheta = zeros(dim_state,dim_controllerParameters);
    % du_dtheta = zeros(dim_control,dim_controllerParameters);

    % initializegradient of loss
    % theta_gradient = zeros(1,dim_controllerParameters);

    % initialize reference state if necessary

    % for k = 1:length(time)-1
        % load current state and current reference
        % X = X_storage(:,end);
        % Xref = Xref_storage(:,end);
 
        % compute the control action (of dimension dim_control x 1)
        % u = controller(...);

        % compute the sensitivity 
        % [dx_dtheta, du_dtheta] = sensitivityComputation(dx_dtheta,...);
        
        % accumulating the gradient of loss wrt controller parameters
        % you need to provide dloss_dx and dloss_du here
        % theta_gradient = theta_gradient + dloss_dx * dx_dtheta + + dloss_du * du_dtheta;

        % integrate the ode dynamics
        % [~,sold] = ode45(@(t,X) dynamics(t,...),[time(k) time(k+1)],X);
        % X_storage = [X_storage sold(end,:)'];

        % integrate the reference system if necessary
        
    % end
    
    % loss is the squared norm of the position tracking error
    % loss = ...
    % loss_hist = [loss_hist loss];

    % update the gradient
    % gradientUpdate = - learningRate * theta_gradient;

    % sanity check
    % if isnan(gradientUpdate)
    %    fprintf('gradient is NAN. Quit.\n');
    %    break;
    % end
   
    % gradient descent
    % theta = theta + gradientUpdate';

    % projection of all parameters to the feasible set
    % the feasible set of parameters in this case is greater than 0.1
    % if any(theta < 0.1)
    %    neg_indicator = (theta < 0.1);
    %    pos_indicator = ~neg_indicator;
    %    theta_min = 0.1*ones(4,1);
    %    theta = neg_indicator.*theta_min + pos_indicator.*theta;
    % end

    % store the parameters
    % param_hist = [param_hist theta];

    % terminate if the total number of iterations is more than maxIterations
    % if itr >= maxIterations
    %    break;
    % end
% end


%% plot trajectory

%% Debug session
% check_dx_dtheta = sum(isnan(dx_dtheta),'all');
% check_du_dtheta = sum(isnan(du_dtheta),'all');
