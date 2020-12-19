% Test data
f = [1, 1, 1, 1];
A = [[-1, 0, 0, 0]; [0, -1, 0, 0]; [0, 0, -1, 0]; [0, 0, 0, -1]];
b = [-1, -1, -1, -1];
F = [[0, 0, -1, 2]; [-1, 0, 1, 0]; [0, -1, 0, 1]];
beq = [0, 0, 0];

% test not form closure
x_1 = [2 2 3 3.5]
y_1 = [1 3 1 2]
angles_1 = [pi/4, -pi/4, 3*pi/4, 5*pi/4]
% test form closure
x_2 = [2 2 3 3];
y_2 = [1 3 1 3];
angles_2 = [pi/4, -pi/4, 3*pi/4, 5*pi/4];

[res, k] = FormClosure(x_1, y_1, angles_1, A, b, f, beq);
if (res == false)
    disp('This grasp does not yield form closure');
else
    disp('This grasp is a form closure');
end