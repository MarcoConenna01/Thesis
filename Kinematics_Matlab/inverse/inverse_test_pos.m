clc 
clear all
close all 

lbr = importrobot("C:\Users\marco\Desktop\Thesis\Kinematics_Matlab\inverse\model\LRM_ARM_simplified_fixed.urdf");
lbr.DataFormat = 'row';
gripper = 'end_effector';
target = [-0.12 0 0.24];

q0 = homeConfiguration(lbr);
qWaypoints = repmat(q0, 2, 1);

gik = generalizedInverseKinematics('RigidBodyTree', lbr, ...
    'ConstraintInputs', {'position','joint'});

posConst = constraintPositionTarget(gripper);
posConst.TargetPosition = target;
posConst.PositionTolerance = 0.001;

limitJointChange = constraintJointBounds(lbr);

[qWaypoints(2,:),solutionInfo] = gik(q0,posConst, limitJointChange);
show(lbr, qWaypoints(2,:), 'PreservePlot', false);

