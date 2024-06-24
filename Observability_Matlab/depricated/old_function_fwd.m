%% COMPLETE FORWARD KINEMATICS OF THE ARM
clc
clear all
close all

% angles 
theta = [0 0 0 0 0 pi/4];

% d
d = [76 0 56 0 49 0];

% extra translations of joints 2 and 4
dd = [70 70];

% a
a = [0 0 0 0 0 0];

% alpha 
alpha = pi/2*[1 -1 1 -1 1 0];

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

% center of mass position wrt. same joint position, x y z
cm = [ 0 0 30; ... %1 DOF
       0 0 20; ...
       0 0 30; ...
       0 0 30; ...
       0 0 30; ...
       0 0 50]; % gripper

% compute position of the end effector
P = End_effector_pos(cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta);

function [posgripper] = End_effector_pos(cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta)

    % base translation and rotation
    A_transl = [1 0 0 x_base; 0 1 0 y_base; 0 0 1 z_base; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_base_alpha) -sin(angle_base_alpha) 0; 0 sin(angle_base_alpha) cos(angle_base_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_base_beta) 0 sin(angle_base_beta) 0; 0 1 0 0; -sin(angle_base_beta) 0 cos(angle_base_beta) 0; 0 0 0 1];
    A_base = A_transl*Aalpha*Abeta;

    % 3rd and 5th joint y translations
    trans_3 = [1 0 0 0; 0 1 0 0; 0 0 1 dd(1); 0 0 0 1];
    trans_5 = [1 0 0 0; 0 1 0 0; 0 0 1 dd(2); 0 0 0 1];

    % end-effector translation and rotation
    A_transl = [1 0 0 x_gripper; 0 1 0 y_gripper; 0 0 1 z_gripper; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_gripper_alpha) -sin(angle_gripper_alpha) 0; 0 sin(angle_gripper_alpha) cos(angle_gripper_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_gripper_beta) 0 sin(angle_gripper_beta) 0; 0 1 0 0; -sin(angle_gripper_beta) 0 cos(angle_gripper_beta) 0; 0 0 0 1];
    A = Aalpha*Abeta*A_transl;
    
    % Compute transformation matrices
    A01 = buildHD(theta(1), alpha(1), d(1), a(1));
    A12 = buildHD(theta(2), alpha(2), d(2), a(2));
    A23 = buildHD(theta(3), alpha(3), d(3), a(3));
    A34 = buildHD(theta(4), alpha(4), d(4), a(4));
    A45 = buildHD(theta(5), alpha(5), d(5), a(5));
    A56 = buildHD(theta(6), alpha(6), d(6), a(6));
    
    % Compute positions of each joint
    T02 = A_base * A01;
    T03 = T02 * A12 * trans_3;
    T04 = T03 * A23;
    T05 = T04 * A34 * trans_5 ;
    T06 = T05 * A45;
    Tgripper = T06 * A56;
    Tgripper = Tgripper * A;

    % Extract joint positions
    pos1 = A_base(1:3, 4);
    pos2 = T02(1:3, 4);
    pos3 = T03(1:3, 4);
    pos4 = T04(1:3, 4);
    pos5 = T05(1:3, 4);
    pos6 = T06(1:3, 4);
    posgripper = Tgripper(1:3, 4);

    % Compute positions of centers of mass of each joint
    cm01 = A_base * [1 0 0 cm(1,1); 0 1 0 cm(1,2); 0 0 1 cm(1,3); 0 0 0 1];
    cm02 = T03 * [1 0 0 cm(2,1); 0 1 0 cm(2,2); 0 0 1 cm(2,3) - dd(1); 0 0 0 1];    
    cm03 = T03 * [1 0 0 cm(3,1); 0 1 0 cm(3,2); 0 0 1 cm(3,3); 0 0 0 1];    
    cm04 = T05 * [1 0 0 cm(4,1); 0 1 0 cm(4,2); 0 0 1 cm(4,3) - dd(2); 0 0 0 1];   
    cm05 = T05 * [1 0 0 cm(5,1); 0 1 0 cm(5,2); 0 0 1 cm(5,3); 0 0 0 1];   
    cm06 = Tgripper * [1 0 0 cm(6,1); 0 1 0 cm(6,2); 0 0 1 cm(6,3) - z_gripper; 0 0 0 1];    

    % Extract centers of mass positions
    poscm01 = cm01(1:3, 4);
    poscm02 = cm02(1:3, 4);
    poscm03 = cm03(1:3, 4);
    poscm04 = cm04(1:3, 4);
    poscm05 = cm05(1:3, 4);
    poscm06 = cm06(1:3, 4);
    
    % Vector of joints position
    posx = [0 pos1(1) pos2(1) pos3(1) pos4(1) pos5(1) pos6(1) posgripper(1)];
    posy = [0 pos1(2) pos2(2) pos3(2) pos4(2) pos5(2) pos6(2) posgripper(2)];
    posz = [0 pos1(3) pos2(3) pos3(3) pos4(3) pos5(3) pos6(3) posgripper(3)];

    % Vector of cm positions
    posxx = [poscm01(1) poscm02(1) poscm03(1) poscm04(1) poscm05(1) poscm06(1)];
    posyy = [poscm01(2) poscm02(2) poscm03(2) poscm04(2) poscm05(2) poscm06(2)];
    poszz = [poscm01(3) poscm02(3) poscm03(3) poscm04(3) poscm05(3) poscm06(3)];
    
    % Plot the points as red big dots with a line connecting them
    figure(2);
    scatter3(posx, posy, posz, 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
    text(posx, posy, posz, ['0';'1';'2';'3';'4';'5';'6';'7'])
    grid on;
    axis equal
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Coordinates');
    hold on;
    
    % Plot the centers of mass
    scatter3(posxx, posyy, poszz, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    text(posxx, posyy, poszz, ['cm01';'cm02';'cm03';'cm04';'cm05';'cm06'])
    
    % Plot reference frames at each joint
    scale = 20; % Scale for the quiver arrows
    plotFrame(A_base, scale);
    plotFrame(T02, scale);
    plotFrame(T03, scale);
    plotFrame(T04, scale);
    plotFrame(T05, scale);
    plotFrame(T06, scale);
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