function desired = command_line(t)

height = 5;

v = 4;
desired.x = [v*t, 0, -height]';
desired.v = [v, 0, 0]';
desired.x_2dot = [0, 0, 0]';
desired.x_3dot = [0, 0, 0]';
desired.x_4dot = [0, 0, 0]';

w = 0;
desired.w = w;
desired.w_dot = 0;

desired.yaw = w*t;

desired.b1 = [cos(w * t), sin(w * t), 0]';
desired.b1_dot = w * [-sin(w * t), cos(w * t), 0]';
desired.b1_2dot = w^2 * [-cos(w * t), -sin(w * t), 0]';

end