close all;
clear all;

addpath('Common\');
addpath('mex\');
import casadi.*

%% Simulation mode
param.generateVideo = true;

% options in this script
% grad_algo = 'GD';
% options include
% gradient descent: 'GD' (need to configure learningRate);
% gradient descent with momentum: 'GDM' (need to configure learningRate and momentum);
% line search: 'LS' (no parameter needed);
% Gauss-Newton: 'GN' (no parameter needed);
% Levenberg-Marquardt: 'LM' (need to configure the damping coefficient)
% BFGS: 'BFGS' (no parameter needed)

grad_algo = 'GD';
switch(grad_algo)
    case 'GD'
        learningRate = 0.001;
    case 'GDM'
        learningRate = 0.001;
        momemtum = 0.5;
    case 'LM'
        damping = 20;
    case 'BFGS'
        % initialize the variables for storage and iterations
        H = eye(12); % initialization of the approximated Hessian inverse in BFGS
        curvature_storage = [];
        theta_gradient_storage = [];
end

%% Simulation parameters
param.dt = 0.0025;
t = 0:param.dt:10;
N = length(t);

% moment of inertia
J1 = 0.0820;
J2 = 0.0845;
J3 = 0.1377;
param.J = diag([J1, J2, J3]);

% mass
param.m = 4.34;

param.g = 9.81;


%% Initialize controller gains
k.x = 16*ones(3,1);
k.v = 5.6*ones(3,1);
k.R = 8.81*ones(3,1);
k.W = 2.54*ones(3,1);

itr = 0;
loss_prev = 1e6;

if param.generateVideo
    video_obj = VideoWriter('video1.mp4','MPEG-4');
    video_obj.FrameRate = 15;
    open(video_obj);
end

loss_history = [];
rmse_history = [];
param_hist = [];
gradientUpdate = zeros(1,12);
while 1
    itr = itr + 1;
    %% Initial conditions
    p0 = [0, 0, 0]';
    v0 = [0, 0, 0]';
    R0 = eye(3);
    omega0 = [0, 0, 0.001]';

    X_storage = [p0; v0; omega0; reshape(R0,9,1)];
    desiredPosition_storage = [];

    dx_dtheta = zeros(18,12,N+1);
    du_dtheta = zeros(4,12,N);

    loss = 0;
    theta_gradient = zeros(1,12);

    e.x = zeros(3,N);
    e.v = zeros(3,N);
    e.R = zeros(3,N);
    e.W = zeros(3,N);

    duration = [];

    for i = 1:N
        % load current state
        X = X_storage(:,end);

        % generate desired states
        desired = command((i-1) * param.dt);
        desiredPosition_storage = [desiredPosition_storage desired.x];

        % compute control actions
        [f, M, err] = geometric_controller(X, desired, k, param);
        u = [f;M];

        % store the errors
        e.x(:,i) = err.x;
        e.v(:,i) = err.v;
        e.R(:,i) = err.R;
        e.W(:,i) = err.W;

        % compute the sensitivity
        t_seed = tic;
        [dx_dtheta(:,:,i+1), du_dtheta(:,:,i)] = sensitivityComputation(dx_dtheta(:,:,i), X, u, desired, param, k);
        duration = [duration toc(t_seed)];

        % compute the error
        norm_ex = norm(err.x);
        norm_eR = norm(err.R);

        % the loss is position tracking error norm square
        loss = loss + norm(err.x)^2;
        theta_gradient = theta_gradient + 2*[err.x;zeros(15,1)]' * dx_dtheta(:,:,i);

        % integrate the ode dynamics
        [~, Xsol] = ode45(@(t, XR) dynamics(t, XR, u, param), param.dt*[i-1 i], X, odeset('RelTol', 1e-6, 'AbsTol', 1e-6));
        
        % store the new state
        X_storage = [X_storage Xsol(end,:)'];
    end

    % compute the RMSE
    RMSE = sqrt(1/N * loss);
    fprintf('Iteration %d, current loss is %f (RMSE %f).\n',itr,loss,RMSE);

    % hard break
    if itr>=100
        break
    end

    loss_prev = loss;
    loss_history = [loss_history; loss];
    rmse_history = [rmse_history; RMSE];

    % update the gradient
    switch(grad_algo)
        case 'GD'
            gradientUpdate = - learningRate * theta_gradient;
        case 'GDM'
            gradientUpdate = momemtum * gradientUpdate - learningRate * theta_gradient;
        case 'LS'
            % compute learning rate
            grad_denom = 0;
            grad_numer = 0;
            for i = 1:N
                grad_numer = grad_numer + e.x(:,i)'*[eye(3) zeros(3,15)]*dx_dtheta(:,:,i)*theta_gradient';
                grad_denom = grad_denom + theta_gradient*dx_dtheta(:,:,i)'*[eye(3) zeros(3,15);zeros(15,18)]*dx_dtheta(:,:,i)*theta_gradient';
            end
            gradientUpdate = - grad_numer / grad_denom * theta_gradient;
        case 'GN'
            % compute gradient direction
            hessian = 0;
            vector = 0;
            for i = 1:N
               hessian = hessian + dx_dtheta(:,:,i)'*dx_dtheta(:,:,i); %12x12
               vector = vector - dx_dtheta(:,:,i)'*[eye(3) zeros(3,15)]'*e.x(:,i);  %12x1
            end
            % vector = -0.5*theta_gradient in this case
            gradientUpdate = (inv(hessian)*vector)';
        case 'LM'
           % compute gradient direction
            hessian = 0;
            vector = 0;
            for i = 1:N
               hessian = hessian + dx_dtheta(:,:,i)'*dx_dtheta(:,:,i); %12x12
               vector = vector - dx_dtheta(:,:,i)'*[eye(3) zeros(3,15)]'*e.x(:,i);  %12x1
            end
            % vector = -0.5*theta_gradient in this case
            gradientUpdate = (inv(hessian + damping * eye(size(hessian)))*vector)';
        case 'BFGS'
            theta_gradient_storage = [theta_gradient_storage;theta_gradient];
            % BFGS update descent direction
            p = (H * theta_gradient')';
            
            % do the line search
            grad_denom = 0;
            grad_numer = 0;

            BFGS_grad_denom = 0;
            BFGS_grad_numer = 0;
            for i = 1:N
                BFGS_grad_numer = BFGS_grad_numer + (e.x(:,i))'*[eye(3) zeros(3,15)]*(dx_dtheta(:,:,i)*p');
                BFGS_grad_denom = BFGS_grad_denom + (dx_dtheta(:,:,i)*p')'*(dx_dtheta(:,:,i)*p');
            end
            % gradient descent
            BFGS_learningRate = BFGS_grad_numer / BFGS_grad_denom;

            gradientUpdate = - BFGS_learningRate * p;
    end

    % update the parameters
    k.x = k.x +  gradientUpdate(1:3)';
    k.v = k.v +  gradientUpdate(4:6)';
    k.R = k.R +  gradientUpdate(7:9)';
    k.W = k.W +  gradientUpdate(10:12)';
    
    % projection of all parameters to be > 0.5
    if any(k.x < 0.5)
        neg_indicator = (k.x < 0.5);
        pos_indicator = ~neg_indicator;
        k_default = 0.5*ones(3,1);
        k.x = neg_indicator.*k_default + pos_indicator.*k.x;
    end

    if any(k.v < 0.5)
        neg_indicator = (k.v < 0.5);
        pos_indicator = ~neg_indicator;
        k_default = 0.5*ones(3,1);
        k.v = neg_indicator.*k_default + pos_indicator.*k.v;
    end

    if any(k.R < 0.5)
        neg_indicator = (k.R < 0.5);
        pos_indicator = ~neg_indicator;
        k_default = 0.5*ones(3,1);
        k.R = neg_indicator.*k_default + pos_indicator.*k.R;
    end

    if any(k.W < 0.5)
        neg_indicator = (k.W < 0.5);
        pos_indicator = ~neg_indicator;
        k_default = 0.5*ones(3,1);
        k.W = neg_indicator.*k_default + pos_indicator.*k.W;
    end

    % store the parameters
    param_hist = [param_hist [k.x; k.v; k.R; k.W]];

    % post-processing for BFGS
    if strcmp(grad_algo, 'BFGS')
        % BFGS update the Hessian Inverse update
        if itr >= 2
            s = param_hist(:,end) - param_hist(:,end-1);
            y = theta_gradient_storage(end,:) - theta_gradient_storage(end-1,:);
            rho = 1/(y*s);
            H = (eye(12) - rho * s * y)*H*(eye(12) - rho * y' * s') + rho * s * s';
            curvature_storage = [curvature_storage y*s];

        end
        if exist('rho')
            y*s
            if y*s <= 0
                fprintf('Terminate becuase the curvature in BFGS turns negative');
                break;
            end
        end
    end

    % visualization for movie
    if param.generateVideo
        figure(4001);
        set(gcf,'Position',[472 320 950 455]);
        set(gcf,'color','w');


        % x tracking
        subplot(3,3,[1,2]);
        plot(t,X_storage(1,1:end-1),'DisplayName','actual','LineWidth',1.5);
        hold on;
        plot(t,desiredPosition_storage(1,:),':','DisplayName','desired','LineWidth',1.5);
        ylabel('x [m]');
        grid on;
        h_lgd = legend;
        set(h_lgd,'Position',[0.3811 0.8099 0.1097 0.0846],'FontSize',10);
        set(gca,'FontSize',10);

        % y tracking
        subplot(3,3,[4,5]);
        plot(t,X_storage(2,1:end-1),'LineWidth',1.5);
        hold on;
        plot(t,desiredPosition_storage(2,:),':','LineWidth',1.5);
        ylabel('y [m]');
        grid on;
        set(gca,'FontSize',10);

        % z tracking
        subplot(3,3,[7,8]);
        plot(t,X_storage(3,1:end-1),'LineWidth',1.5);
        hold on;
        plot(t,desiredPosition_storage(3,:),':','LineWidth',1.5);
        ylabel('z [m]');
        xlabel('time [s]');
        grid on;
        set(gca,'FontSize',10);

        % rmse
        subplot(3,3,[3;6;9]);
        plot(rmse_history,'LineWidth',1.5);
        hold on;
        grid on;
        stem(length(rmse_history),rmse_history(end),'Color',[0 0.4470 0.7410]);

        xlim([0 100]);
        ylim([0 rmse_history(1)*1.1]);
        text(50,0.3,['iteration = ' num2str(length(rmse_history))],'FontSize',12);
        xlabel('iterations');
        ylabel('RMSE [m]');
        set(gca,'FontSize',10);
        plotedit(gca,'on');
        plotedit(gca,'off');

        drawnow;

        frame = getframe(gcf);
        writeVideo(video_obj,frame);
        clf
    end

end
if param.generateVideo
    close(video_obj);
end

%% Plots
linetype = 'k';
linewidth = 1;
xlabel_ = 'time (s)';

figure(2);
plot_3x1(t, e.R, '', xlabel_, 'e_R', linetype, linewidth)
set(gca, 'FontName', 'Times New Roman');

figure(3);
plot_3x1(t, e.x, '', xlabel_, 'e_x', linetype, linewidth)
set(gca, 'FontName', 'Times New Roman');

figure(4);
plot_3x1(t, X_storage(1:3,1:end-1), '', xlabel_, 'x', linetype, linewidth)
plot_3x1(t, desiredPosition_storage, '', xlabel_, 'x', 'r:', linewidth)
set(gca, 'FontName', 'Times New Roman');

% %% Debug session
% check_dx_dtheta = sum(isnan(dx_dtheta),'all');
% check_du_dtheta = sum(isnan(du_dtheta),'all');