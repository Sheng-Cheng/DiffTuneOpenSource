% This script runs a simulation of controlling a 2D Dubin's car model with
% controller parameters updated using DiffTune
% Sheng Cheng, Sept 2022

% vehicle parameters include
% m: mass
% J: moment of inertia

% states include
% x: horizontal position
% y: vertical position
% theta: yaw angle
% v: velocity along the body forward direction (body x)
% omega: angular rate of the yaw angle

% control includes
% F: force along the body forward direction (body x)
% M: moment on the vehicle

close all;
clear all;

addpath('mex\');
import casadi.*

%% video options
param.generateVideo = true;
if param.generateVideo
    video_obj = VideoWriter('DubinCar.mp4','MPEG-4');
    video_obj.FrameRate = 15;
    open(video_obj);
end

%% Simulation parameters
dt = 0.01;
time = 0:dt:6.28;

% mass
m = 1;
% inertia
J = 1;

%% Initialize controller gains
kp = 5; 
kd = 5; 
ktheta = 5; 
komega = 5; 
k_vec = [kp;kd;ktheta;komega];

%% Desired trajectory parameterization
% generate the desired trajectories using linear and angular velocities
% circular trajectory, where the forward speed vd is a constant, and yaw
% rate omegad equals vd/circleRadius
vd = 1*ones(length(time)-1,1); % linear speed 1 m/s
vd_dot = zeros(size(vd));
omegad = 1*ones(length(time)-1,1); % on a circle with radius 1 m
omegad_dot = zeros(size(omegad));

%% Initialize variables for DiffTune iterations
learningRate = 2;
totalIterations = 100;
itr = 0;

loss_hist = [];
rmse_history = [];
param_hist = [];
gradientUpdate = zeros(4,1);

%% DiffTune iterations
while (1)
    itr = itr + 1;

    % initialize state
    X_storage = zeros(5,1);
    
    % initialize sensitivity
    dx_dtheta = zeros(5,4);
    du_dtheta = zeros(2,4);

    % initializegradient of loss
    theta_gradient = zeros(1,4);

    % initialize reference state based on the desired trajectory 
    Xref_storage = [X_storage(1:3);vd(1);omegad(1)]; 

    for k = 1:length(time)-1
        % load current state and current reference
        X = X_storage(:,end);
        Xref = Xref_storage(:,end);
 
        % compute the control action
        u = controller(X,Xref,vd_dot(k),omegad_dot(k),m,J,k_vec);

        % compute the sensitivity 
        [dx_dtheta, du_dtheta] = sensitivityComputation(dx_dtheta,X,Xref,vd_dot(k),omegad_dot(k),u,m,J,k_vec,dt);
        
        % accumulating the gradient of loss wrt controller parameters
        theta_gradient = theta_gradient + 2 * [X(1) - Xref(1) X(2) - Xref(2) 0 0 0] * dx_dtheta;

        % integrate the ode dynamics
        [~,sold] = ode45(@(t,X) dynamics(t,X,u,[m;J]),[time(k) time(k+1)],X);
        X_storage = [X_storage sold(end,:)'];

        % integrate the reference system to obtain the reference state
        [~,solref] = ode45(@(t,X) dynamics(t,X, [vd_dot(k) omegad_dot(k)] ,[m;J]),[time(k) time(k+1)],Xref);
        Xref_storage = [Xref_storage solref(end,:)'];
        
    end
    % loss is the squared norm of the position tracking error
    loss = trace([X_storage(:,1:end)-Xref_storage(:,1:end)]'*diag([1 1 0 0 0]) * [X_storage(:,1:end)-Xref_storage(:,1:end)]);
    loss_hist = [loss_hist loss];

    % store the RMSE
    rmse_history = [rmse_history sqrt(mean(diag([X_storage(:,1:end)-Xref_storage(:,1:end)]'*diag([1 1 0 0 0]) * [X_storage(:,1:end)-Xref_storage(:,1:end)])))];

    % update the gradient
    gradientUpdate = - learningRate * theta_gradient;

    if isnan(gradientUpdate)
        fprintf('gradient is NAN. Quit.\n');
        break;
    end
   
    % gradient descent
    k_vec = k_vec + gradientUpdate';

    % projection of all parameters to be > 0.1
    if any(k_vec < 0.1)
        neg_indicator = (k_vec < 0.1);
        pos_indicator = ~neg_indicator;
        k_default = 0.1*ones(4,1);
        k_vec = neg_indicator.*k_default + pos_indicator.*k_vec;
    end

   
    % store the parameters
    param_hist = [param_hist k_vec];
    
    subplot(1,3,[1 2]);
    plot(Xref_storage(1,:),Xref_storage(2,:),'DisplayName','desired');
    hold on;
    % L1 off
    plot(X_storage(1,:),X_storage(2,:),'DisplayName','PD only');

    axis equal
    xlabel('x [m]');
    ylabel('y [m]');
    legend;

    kp = k_vec(1);
    kd = k_vec(2);
    ktheta = k_vec(3);
    komega = k_vec(4);

    text(-0.5,1.6,['itr = ' num2str(itr)]);
    text(-0.5,1.5,['kp = ' num2str(kp) ', grad = ' num2str(theta_gradient(1))]);
    text(-0.5,1.4,['kd = ' num2str(kd) ', grad = ' num2str(theta_gradient(2))]);
    text(-0.5,1.3,['ktheta = ' num2str(ktheta) ', grad = ' num2str(theta_gradient(3))]);
    text(-0.5,1.2,['komega = ' num2str(komega) ', grad = ' num2str(theta_gradient(4))]);
    text(-0.5,1.1,['loss = ' num2str(loss)]);
    
    % rmse
    subplot(1,3,3);
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

    set(gcf,'Position',[360 278 714.3333 420]);

    drawnow;

    % visualization for movie
    if param.generateVideo
        frame = getframe(gcf);
        writeVideo(video_obj,frame);
        clf
    end

    % hard break
    if itr >= totalIterations
        break;
    end
end

if param.generateVideo
    close(video_obj);
end

%% plot trajectory by axis
figure;
subplot(2,1,1);
plot(Xref_storage(1,:),'DisplayName','desired');
hold on;
plot(X_storage(1,:),'DisplayName','PD');
legend;
ylabel('x [m]');

subplot(2,1,2);
plot(Xref_storage(2,:),'DisplayName','disred');
hold on;
plot(X_storage(2,:),'DisplayName','PD');
legend;
ylabel('y [m]');

% %% Debug session
% check_dx_dtheta = sum(isnan(dx_dtheta),'all');
% check_du_dtheta = sum(isnan(du_dtheta),'all');
