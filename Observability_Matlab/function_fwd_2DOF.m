eps = 0.00001;

% angles 
theta = [0 0];
% d
d = [76 0];
% a
a = [0 0];
% alpha 
alpha = pi/2*[1 0];
%gripper and base
angle_gripper_alpha = -pi/2;
angle_gripper_beta = 0;
x_gripper = 0;
y_gripper = 0;
z_gripper = 129;
angle_base_alpha = 0;
angle_base_beta = pi/3;
x_base = 0;
y_base = 0;
z_base = 227;


P = End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta)



function [posgripper] = End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta)

    % end-effector translation and rotation
    A_transl = [1 0 0 x_gripper; 0 1 0 y_gripper; 0 0 1 z_gripper; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_gripper_alpha) -sin(angle_gripper_alpha) 0; 0 sin(angle_gripper_alpha) cos(angle_gripper_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_gripper_beta) 0 sin(angle_gripper_beta) 0; 0 1 0 0; -sin(angle_gripper_beta) 0 cos(angle_gripper_beta) 0; 0 0 0 1];
    A = Aalpha*Abeta*A_transl;
    
    % base translation and rotation
    A_transl = [1 0 0 x_base; 0 1 0 y_base; 0 0 1 z_base; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_base_alpha) -sin(angle_base_alpha) 0; 0 sin(angle_base_alpha) cos(angle_base_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_base_beta) 0 sin(angle_base_beta) 0; 0 1 0 0; -sin(angle_base_beta) 0 cos(angle_base_beta) 0; 0 0 0 1];
    A_base = A_transl*Aalpha*Abeta;
    
    % Compute transformation matrices
    A01 = buildHD(theta(1), alpha(1), d(1), a(1));
    A12 = buildHD(theta(2), alpha(2), d(2), a(2));

    % Compute positions of each joint
    T01 = A_base * A01;
    T02 = T01 * A12;
    Tgripper = T02 * A;

    % Extract joint positions
    pos0 = A_base(1:3, 4);
    pos1 = T01(1:3, 4);
    pos2 = T02(1:3, 4);
    posgripper = Tgripper(1:3, 4);
    
    posx = [0 pos0(1) pos1(1) pos2(1) posgripper(1)];
    posy = [0 pos0(2) pos1(2) pos2(2) posgripper(2)];
    posz = [0 pos0(3) pos1(3) pos2(3) posgripper(3)];
    
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
    plotFrame(Tgripper, scale);
end

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