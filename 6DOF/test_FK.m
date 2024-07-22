%% PROBLEM!!!! NOT CONSISTENT WITH LBR!!!

clear all
close all
clc

lbr = importrobot("C:\Users\marco\Documents\GitHub\Thesis\6DOF\LRM_ARM_testing\urdf\LRM_ARM_testing2.urdf");
lbr.DataFormat = 'row';
gripper = 'End_effector';

v = zeros(59,1);
theta = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2];

tform = getTransform(lbr, theta, 'End_effector');
P = tform(1:3,4);
tform = getTransform(lbr, theta, 'Link1');
P1 = tform(1:3,4);
tform = getTransform(lbr, theta, 'Link2');
P2 = tform(1:3,4);
tform = getTransform(lbr, theta, 'Link3');
P3 = tform(1:3,4);
tform = getTransform(lbr, theta, 'Link4');
P4 = tform(1:3,4);
tform = getTransform(lbr, theta, 'Link5');
P5 = tform(1:3,4);
tform = getTransform(lbr, theta, 'Link6');
P6 = tform(1:3,4);

Pt = FK(theta, v);
Pt = Pt/1000;

Pt = [Pt(7:end); Pt(1:3)];

Purdf = [P1; P2; P3; P4; P5; P6; P];


figure()
plot3(P(1), P(2), P(3), '.', 'MarkerSize', 20, "Color", "r")
hold on
plot3(Pt(1), Pt(2), Pt(3), '.', 'MarkerSize', 5, "Color", "b")
plot3(Pt(7), Pt(8), Pt(9), '.', 'MarkerSize', 20, "Color", "b")
plot3(Pt(10), Pt(11), Pt(12), '.', 'MarkerSize', 20, "Color", "b")
plot3(Pt(13), Pt(14), Pt(15), '.', 'MarkerSize', 20, "Color", "b")
plot3(Pt(16), Pt(17), Pt(18), '.', 'MarkerSize', 20, "Color", "b")
plot3(Pt(19), Pt(20), Pt(21), '.', 'MarkerSize', 20, "Color", "b")
plot3(Pt(22), Pt(23), Pt(24), '.', 'MarkerSize', 20, "Color", "b")
