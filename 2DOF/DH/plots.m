%% PLOT POSITION
load lrm_arm_marker_03_07
position_arm = [pose_position_x' pose_position_y' pose_position_z'];
load lrm_base_marker_03_07.mat
position_base = [pose_position_x' pose_position_y' pose_position_z'];

position = 1000*(position_arm - position_base);
seconds = linspace(0,200, 10000);
plot(seconds, position(1:10000,1), LineWidth=1.5, Color="r")
hold on
plot(seconds, position(1:10000,2), LineWidth=1.5, Color="b")
plot(seconds, position(1:10000,3), LineWidth=1.5, Color="g")
title("TCP Position Data for the 2DOF Test")
xlabel("Time [s]")
ylabel("Position w.r.t Base [mm]")
grid on
legend("x","y","z")