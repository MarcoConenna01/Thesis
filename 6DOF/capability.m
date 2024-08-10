%% CAPABILITY MAP
clc
clear all

arm = importrobot("C:\Users\marco\Documents\GitHub\Thesis\6DOF\LRM_ARM_urdf\urdf\LRM_ARM_urdf.urdf");
arm.DataFormat = 'row';
gripper = 'End_effector';

field = linspace(deg2rad(-100),deg2rad(100),10);
n = length(field);
n_tot = 10^6;
counter = 1;

xx = linspace(1,1000,100); %-500:500;
yy = linspace(1,1000,100); %-500:500;
zz = linspace(1,1000,100); %-300:700;
map = zeros(length(xx),length(yy),length(zz));

for i = 1:n
    for j = 1:n
        for k = 1:n
            for l = 1:n
                for m = 1:n
                    for o = 1:n
                        theta = [field(i) field(j) field(k) field(l) field(m) field(o)];
                        tform = getTransform(arm, theta, 'End_effector');
                        P = tform(1:3,4);
                        pos = P*1000;
                        x = round(pos(1)/10) + 50;
                        y = round(pos(2)/10) + 50;
                        z = round(pos(3)/10) + 30;
                        map(x,y,z) = map(x,y,z) + 1;
                        counter = counter + 1
                    end
                end
            end
        end
    end
end

%% 
counter = 1;
for i = 1:100
    for j = 1:100
        for k = 30:40
            if map(i,j,k) ~= 0
                capability_map(counter,:) = [i*10-500,j*10-500,k*10-300,map(i,j,k)];
                counter = counter + 1
            end
        end
    end
end
capability_map(:,4) = capability_map(:,4)/max(capability_map(:,4));

%% Plots
figure

% Plot the capability map
show(arm,'Frames','off')
hold on
scatter3(capability_map(1:10:end, 1)/1000, capability_map(1:10:end, 2)/1000, capability_map(1:10:end, 3)/1000, ...
         20, capability_map(1:10:end, 4), 'filled', 'MarkerFaceAlpha', 1)
set(gca, 'ColorScale');
colorbar;
colormap(flipud(jet));
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Capability Map with Robot Arm');
axis equal;
hold off