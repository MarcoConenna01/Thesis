clear all
close all
clc

lbr = importrobot("C:\Users\marco\Documents\GitHub\Thesis\6DOF\LRM_ARM_urdf\urdf\LRM_ARM_urdf.urdf");
lbr.DataFormat = 'row';
gripper = 'End_effector';

v = zeros(59,1);
theta = [0 0 0 0 0 0];

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

Pt = FK_ln(theta, v);

Pt = [Pt(7:end); Pt(1:3)];

%Purdf = [P1; P2; P3; P4; P5; P6; P];
Purdf = P;

error = Purdf*1000 - Pt
