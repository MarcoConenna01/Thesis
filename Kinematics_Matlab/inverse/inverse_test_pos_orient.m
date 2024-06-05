clc 
clear all
close all

load rotations2
lbr = importrobot("C:\Users\marco\Desktop\Thesis\Kinematics_Matlab\inverse\model\LRM_ARM_60_2_2.urdf");
lbr.DataFormat = 'row';
gripper = 'end_effector';
%target = [0.3 0.15 0];
target = [-0.10 0 0.24];

q0 = homeConfiguration(lbr);
qWaypoints = repmat(q0, 2, 1);

gik = generalizedInverseKinematics('RigidBodyTree', lbr, ...
    'ConstraintInputs', {'position','joint','orientation'});
gik.SolverParameters.MaxIterations = 15000;
gik.SolverParameters.MaxTime = 60;

posConst = constraintPositionTarget(gripper);
posConst.TargetPosition = target;
posConst.PositionTolerance = 0;

limitJointChange = constraintJointBounds(lbr);

fixOrientation = constraintOrientationTarget(gripper); %orientation
fixOrientation.OrientationTolerance = deg2rad(15);

for i = 1:15
i
figure(i)
fixOrientation.TargetOrientation = rotations(i,:);
[qWaypoints(2,:),solutionInfo] = gik(q0,posConst, limitJointChange,fixOrientation);
show(lbr, qWaypoints(2,:), 'PreservePlot', false);
hold on
scatter3(target(1),target(2),target(3),'filled')
result(i) = solutionInfo.ExitFlag;
errors(i,:) = getfield(solutionInfo.ConstraintViolations,'Violation',{1});
end