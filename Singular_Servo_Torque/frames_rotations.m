clc
clear all

load weight2_20240712_181650.mat

pose = [pose_orientation_w' pose_orientation_x' pose_orientation_y' pose_orientation_z'];

q0 = mean(pose(5000:6000,:));
R0 = quaternionToRotationMatrix(q0);
q1 = mean(pose(122:241,:));
R1 = quaternionToRotationMatrix(q1);
q2 = mean(pose(600:1980,:));
R2 = quaternionToRotationMatrix(q2);
q3 = mean(pose(2400:3600,:));
R3 = quaternionToRotationMatrix(q3);

z0 = R0(:, 3);  % Z-axis vector of frame 1 after rotation
z1 = R1(:, 3);  % Z-axis vector of frame 2 after rotation
z2 = R2(:, 3);  % Z-axis vector of frame 3 after rotation
z3 = R3(:, 3);  % Z-axis vector of frame 4 after rotation

cos_angle1 = dot(z0, z1);
cos_angle2 = dot(z0, z2);
cos_angle3 = dot(z0, z3);

angle_rad1 = acos(cos_angle1);
angle_rad2 = acos(cos_angle2);
angle_rad3 = acos(cos_angle3);

angle_deg1 = rad2deg(acos(cos_angle1));
angle_deg2 = rad2deg(acos(cos_angle2));
angle_deg3 = rad2deg(acos(cos_angle3));

m_battery = 0.575;
m_beam = 0.0616;
cm_beam = 0.085;

t1 = (cm_beam*m_beam + m_battery*0.15)*9.81;
t2 = (cm_beam*m_beam + 2*m_battery*0.15)*9.81;
t3 = (cm_beam*m_beam + 3*m_battery*0.15)*9.81;

k1 = (angle_rad1) / (t1);
k2 = (angle_rad2 - angle_rad1) / (t2-t1);
k3 = (angle_rad3 - angle_rad2) / (t3-t2);

weights = 1000*[0 cm_beam*m_beam + m_battery*0.15, cm_beam*m_beam + 2*m_battery*0.15, cm_beam*m_beam + 3*m_battery*0.15];
plot([0 t1 t2 t3], [0 angle_rad1 angle_rad2 angle_rad3], "LineWidth",2)
hold on
scatter([0 t1 t2 t3], [0 angle_rad1 angle_rad2 angle_rad3],50,"filled")
xlabel("Torque [Nm]")
ylabel("Angle Deflection [rad]")
grid on
title("Elastic Angular Deformation of the AGF A80BHMF")

function R = quaternionToRotationMatrix(q)
    % Extract quaternion components
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    
    % Compute rotation matrix
    R = [1 - 2*y^2 - 2*z^2,   2*x*y - 2*w*z,       2*x*z + 2*w*y;
         2*x*y + 2*w*z,       1 - 2*x^2 - 2*z^2,   2*y*z - 2*w*x;
         2*x*z - 2*w*y,       2*y*z + 2*w*x,       1 - 2*x^2 - 2*y^2];
end