clc
clearvars
close all
load goodconf.mat

%% Measurements
load arm_pose_400_20240715_124931.mat
position_arm = [pose_position_x' pose_position_y' pose_position_z'];

load lrm_base_marker_400_20240715_115504.mat
position_base = [pose_position_x' pose_position_y' pose_position_z'];

position = 1000*(position_arm - position_base);
position_measured = zeros(100,3);
position_measured_rotated = zeros(100,3);
orientation = [pose_orientation_w' pose_orientation_x'  pose_orientation_y' pose_orientation_z'];

for i = 1:393
    upperlimit = 1110 + 750*(i-1);
    lowerlimit = upperlimit - 100;
    position_measured(i,:) = [mean(position(lowerlimit:upperlimit,1)),mean(position(lowerlimit:upperlimit,2)),mean(position(lowerlimit:upperlimit,3))];
    orientation_measured = [mean(orientation(lowerlimit:upperlimit,1)),mean(orientation(lowerlimit:upperlimit,2)),mean(orientation(lowerlimit:upperlimit,3)), mean(orientation(lowerlimit:upperlimit,4))];
    rotation_matrix = quat2rotm(orientation_measured);
    position_measured_rotated(i,:) = rotation_matrix' * position_measured(i,:)';
end

%% TEST: compared to theoretical position
v = zeros(59,1);
for i = 1:393
    theta = goodconf(i,2:7);
    a = FK(theta,v);
    theoretical_position(i,1:3) = a(1:3);
end
position_measured_rotated(:,1) = - position_measured_rotated(:,1);
position_measured_rotated(:,2) = - position_measured_rotated(:,2);
error = position_measured_rotated - theoretical_position;

