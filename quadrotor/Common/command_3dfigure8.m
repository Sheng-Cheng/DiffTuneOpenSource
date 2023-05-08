function desired = command_3dfigure8(t)
desired.x = [sin(2*t), sin(4*t), sin(t)]';
    desired.v = [2*cos(2*t), 4*cos(4*t), cos(t)]';
    desired.x_2dot = [-4*sin(2*t), -16*sin(4*t), -sin(t)]';
    desired.x_3dot = [-8*cos(2*t), -64*cos(4*t), -cos(t)]';
    desired.x_4dot = [16*sin(2*t), 256*sin(4*t), sin(t)]'; 

desired.yaw = 0;
desired.b1 = [1, 0, 0]';
desired.b1_dot = [0, 0, 0]';
desired.b1_2dot = [0, 0, 0]';

end