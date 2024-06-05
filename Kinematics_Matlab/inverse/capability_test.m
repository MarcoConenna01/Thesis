clc 
clear all
close all


load rotations
lbr = importrobot("C:\Users\marco\Desktop\Thesis\Kinematics_Matlab\inverse\model\LRM_ARM_simplified_fixed.urdf");
lbr.DataFormat = 'row';
gripper = 'end_effector';

q0 = homeConfiguration(lbr);
qWaypoints = repmat(q0, 2, 1);

gik = generalizedInverseKinematics('RigidBodyTree', lbr, ...
    'ConstraintInputs', {'position','joint','orientation'});

posConst = constraintPositionTarget(gripper);

limitJointChange = constraintJointBounds(lbr);

fixOrientation = constraintOrientationTarget(gripper); %orientation
fixOrientation.OrientationTolerance = deg2rad(1);


%% testing for ground with rotations(10,:)

xx = linspace(0,0.5,20);
yy = linspace(-0.4,0.4,8);
errors=zeros(8,8);
counter = 1;
errormin = 100;

for i= 1:length(xx)
    %for j = 1:length(yy)
        
        %posConst.TargetPosition = [xx(i) yy(j) 0];
        posConst.TargetPosition = [xx(i) 0 0];
        fixOrientation.TargetOrientation = rotations(10,:);
        [qWaypoints(2,:),solutionInfo] = gik(q0,posConst, limitJointChange,fixOrientation);
        %show(lbr, qWaypoints(2,:), 'PreservePlot', false);
        errors(i) = getfield(solutionInfo.ConstraintViolations,'Violation',{1});

        xxx(counter) = xx(i);
        yyy(counter) = 0;
        errors2(counter) = errors(i);
        counter = counter+1
    %end
end

%% plot 
figure
scatter(xxx,yyy, 20, errors2, 'filled')
colorbar