%% COMPLETE FORWARD KINEMATICS OF THE ARM WITH TORQUE DUE TO GRAVITY

clc
clear all
close all

% angles 
theta = [0 -pi/6 0 0 pi/2 pi/8];

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

% mass of each link
m = [100 100 100 100 100 200];

% compute position of the end effector
P = End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta);



