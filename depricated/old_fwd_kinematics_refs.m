%% BASIC FORWARD KINEMATICS OF LRM ARM

% angles
theta = [0 0 0 0 0 0];

% d
d = [76 0 136 0 129 0];

% a
a = [0 0 0 0 0 0];

% alpha 
alpha = pi/2*[1 -1 1 -1 1 0];


% end-effector translation and rotation
angle_gripper = -pi/2;
A1 = [1 0 0 0; 0 1 0 129; 0 0 1 0; 0 0 0 1];
A2 = [1 0 0 0; 0 cos(angle_gripper) -sin(angle_gripper) 0; 0 sin(angle_gripper) cos(angle_gripper) 0; 0 0 0 1];
A = A1*A2;

% base translation and rotation
angle_base = pi/3;
A1 = [1 0 0 0; 0 1 0 0; 0 0 1 227; 0 0 0 1];
A2 = [cos(angle_base) 0 sin(angle_base) 0; 0 1 0 0; -sin(angle_base) 0 cos(angle_base) 0; 0 0 0 1];
A_base = A1*A2;

% Compute transformation matrices
A01 = buildHD(theta(1), alpha(1), d(1), a(1));
A12 = buildHD(theta(2), alpha(2), d(2), a(2));
A23 = buildHD(theta(3), alpha(3), d(3), a(3));
A34 = buildHD(theta(4), alpha(4), d(4), a(4));
A45 = buildHD(theta(5), alpha(5), d(5), a(5));
A56 = buildHD(theta(6), alpha(6), d(6), a(6));

% Compute positions of each joint
T01 = A_base * A01;
T02 = T01 * A12;
T03 = T02 * A23;
T04 = T03 * A34;
T05 = T04 * A45;
T06 = T05 * A56;
Tgripper = T06 * A;

% Extract joint positions
pos0 = A_base(1:3, 4);
pos1 = T01(1:3, 4);
pos2 = T02(1:3, 4);
pos3 = T03(1:3, 4);
pos4 = T04(1:3, 4);
pos5 = T05(1:3, 4);
pos6 = T06(1:3, 4);
posgripper = Tgripper(1:3, 4);

posx = [0 pos0(1) pos1(1) pos2(1) pos3(1) pos4(1) pos5(1) pos6(1) posgripper(1)];
posy = [0 pos0(2) pos1(2) pos2(2) pos3(2) pos4(2) pos5(2) pos6(2) posgripper(2)];
posz = [0 pos0(3) pos1(3) pos2(3) pos3(3) pos4(3) pos5(3) pos6(3) posgripper(3)];

% Plot the points as red big dots with a line connecting them
figure(2);
scatter3(posx, posy, posz, 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
grid on;
axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Coordinates');
hold on;

% Plot reference frames at each joint
scale = 20; % Scale for the quiver arrows
plotFrame(A_base, scale);
plotFrame(T01, scale);
plotFrame(T02, scale);
plotFrame(T03, scale);
plotFrame(T04, scale);
plotFrame(T05, scale);
plotFrame(T06, scale);
plotFrame(Tgripper, scale);

% Function to plot the reference frame
function plotFrame(T, scale)
    origin = T(1:3, 4)';
    x_axis = T(1:3, 1)' * scale;
    y_axis = T(1:3, 2)' * scale;
    z_axis = T(1:3, 3)' * scale;
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 'r');
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 'g');
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 'b');
end

% Define the buildHD function
function [A] = buildHD(theta, alpha, d, a)
    A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);...
       sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
       0 sin(alpha) cos(alpha) d;...
       0 0 0 1];
end