clc 
clear all
close all

load rotations2
list_urdf = dir(fullfile('C:\Users\marco\Desktop\Thesis\Kinematics_Matlab\inverse\model\45', '*.urdf'));
RESULT = zeros(521,8);
counter = 1

for iii = 1:length(list_urdf)

    conf_counter = 0;

name = append(list_urdf(1).folder,'\', list_urdf(iii).name);
lbr = importrobot(name);
lbr.DataFormat = 'row';

gripper = 'end_effector';

q0 = homeConfiguration(lbr);
qWaypoints = repmat(q0, 2, 1);

gik = generalizedInverseKinematics('RigidBodyTree', lbr, ...
    'ConstraintInputs', {'position','joint','orientation'});
gik.SolverParameters.MaxIterations = 10000;
gik.SolverParameters.MaxTime = 20;

posConst = constraintPositionTarget(gripper);
posConst.PositionTolerance = 0;

limitJointChange = constraintJointBounds(lbr);

fixOrientation = constraintOrientationTarget(gripper); %orientation

x = linspace(0,0.15,5);
y = linspace(0,0.3,5);

%% ANGLE 60°  0.7854

posConst.TargetPosition = [-0.12 0 0.24];
fixOrientation.TargetOrientation = rotations(1,:);
[qWaypoints(2,:),solutionInfo] = gik(q0,posConst, limitJointChange,fixOrientation);
fixOrientation.OrientationTolerance = deg2rad(60);

if getfield(solutionInfo.ConstraintViolations,'Violation',{1}) < 0.0001 % found a solution
    
    % 1 retrieval
    fixOrientation.OrientationTolerance = deg2rad(10);
    [qWaypoints(2,:),solutionInfo] = gik(q0,posConst, limitJointChange,fixOrientation);
    if getfield(solutionInfo.ConstraintViolations,'Violation',{1}) < 0.0001
    RESULT(counter,1) = 45;
    RESULT(counter,2) = lbr.Bodies{1, 3}.Joint.JointToParentTransform(2,4);
    RESULT(counter,3) = lbr.Bodies{1, 5}.Joint.JointToParentTransform(2,4);
    RESULT(counter,4) = 1;
    RESULT(counter,5) = -0.12;
    RESULT(counter,6) = 0;
    RESULT(counter,7) = 0.24;
    RESULT(counter,8) = getfield(solutionInfo.ConstraintViolations,'Violation',{1}) < 0.0001;
    conf_counter = conf_counter + RESULT(counter,8);
    counter = counter + 1
    end

    % 2 retrieval
    fixOrientation.OrientationTolerance = deg2rad(30);
    fixOrientation.TargetOrientation = rotations(15,:);
    [qWaypoints(2,:),solutionInfo] = gik(q0,posConst, limitJointChange,fixOrientation);
    if getfield(solutionInfo.ConstraintViolations,'Violation',{1}) < 0.0001
    RESULT(counter,1) = 45;
    RESULT(counter,2) = lbr.Bodies{1, 3}.Joint.JointToParentTransform(2,4);
    RESULT(counter,3) = lbr.Bodies{1, 5}.Joint.JointToParentTransform(2,4);
    RESULT(counter,4) = 2;
    RESULT(counter,5) = -0.12;
    RESULT(counter,6) = 0;
    RESULT(counter,7) = 0.24;
    RESULT(counter,8) = getfield(solutionInfo.ConstraintViolations,'Violation',{1}) < 0.0001;
    conf_counter = conf_counter + RESULT(counter,8);
    counter = counter + 1
    end
    
    for i = 1:length(x) 
        for j = length(y)

            posConst.TargetPosition = [x(i) y(i) 0];

            % 3 grabbing
            fixOrientation.OrientationTolerance = deg2rad(10);
            fixOrientation.TargetOrientation = rotations(1,:);
            [qWaypoints(2,:),solutionInfo] = gik(q0,posConst, limitJointChange,fixOrientation);
            RESULT(counter,1) = 45;
            RESULT(counter,2) = lbr.Bodies{1, 3}.Joint.JointToParentTransform(2,4);
            RESULT(counter,3) = lbr.Bodies{1, 5}.Joint.JointToParentTransform(2,4);
            RESULT(counter,4) = 3;
            RESULT(counter,5) = x(i);
            RESULT(counter,6) = y(i);
            RESULT(counter,7) = 0;
            RESULT(counter,8) = getfield(solutionInfo.ConstraintViolations,'Violation',{1}) < 0.0001;
            conf_counter = conf_counter + RESULT(counter,8);
            counter = counter + 1;
        
            % 4 grabbing
            fixOrientation.OrientationTolerance = deg2rad(15);
            fixOrientation.TargetOrientation = rotations(13,:);
            [qWaypoints(2,:),solutionInfo] = gik(q0,posConst, limitJointChange,fixOrientation);
            RESULT(counter,1) = 45;
            RESULT(counter,2) = lbr.Bodies{1, 3}.Joint.JointToParentTransform(2,4);
            RESULT(counter,3) = lbr.Bodies{1, 5}.Joint.JointToParentTransform(2,4);
            RESULT(counter,4) = 4;
            RESULT(counter,5) = x(i);
            RESULT(counter,6) = y(i);
            RESULT(counter,7) = 0;
            RESULT(counter,8) = getfield(solutionInfo.ConstraintViolations,'Violation',{1}) < 0.0001;
            conf_counter = conf_counter + RESULT(counter,8);
            counter = counter + 1
        
            % 5 grabbing
            fixOrientation.OrientationTolerance = deg2rad(15);
            fixOrientation.TargetOrientation = rotations(8,:);
            [qWaypoints(2,:),solutionInfo] = gik(q0,posConst, limitJointChange,fixOrientation);
            RESULT(counter,1) = 45;
            RESULT(counter,2) = lbr.Bodies{1, 3}.Joint.JointToParentTransform(2,4);
            RESULT(counter,3) = lbr.Bodies{1, 5}.Joint.JointToParentTransform(2,4);
            RESULT(counter,4) = 5;
            RESULT(counter,5) = x(i);
            RESULT(counter,6) = y(i);
            RESULT(counter,7) = 0;
            RESULT(counter,8) = getfield(solutionInfo.ConstraintViolations,'Violation',{1}) < 0.0001;
            conf_counter = conf_counter + RESULT(counter,8);
            counter = counter + 1
        end
    end
end

RESULT(counter:counter+2,:) = zeros(3,8);
RESULT(counter,1) = conf_counter;
counter = counter + 3

end