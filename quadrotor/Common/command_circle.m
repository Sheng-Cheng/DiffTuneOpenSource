function desired = command_circle(t)
desired.x = [2*(1-cos(t)), 2*sin(t), 0.1*sin(t)]';
desired.v = [2*sin(t), 2*cos(t), 0.1*cos(t)]';
desired.x_2dot = [2*cos(t), -2*sin(t), -0.1*sin(t)]';
desired.x_3dot = [-2*sin(t), -2*cos(t), -0.1*cos(t)]';
desired.x_4dot = [-2*cos(t), 2*sin(t), 0.1*sin(t)]';

desired.yaw = 0;
desired.b1 = [1, 0, 0]';
desired.b1_dot = [0, 0, 0]';
desired.b1_2dot = [0, 0, 0]';

end