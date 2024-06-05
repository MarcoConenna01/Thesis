clc 
clear all
load end_effector_1_186_2_179

% Define grid parameters
gridResolution = 20; % size of each voxel, 20x20x20 mm
gridExtent = [1200, 1200, 1200]; % extent of the grid in x, y, and z directions
gridmin = [-600, -600, -200]; % minimum value of the grid in x, y, z directions

capability_value = zeros(gridExtent(1)/gridResolution,gridExtent(2)/gridResolution,gridExtent(3)/gridResolution);

for i = 1:length(end_effector)
    x = round(end_effector(1, i)/gridResolution) - gridmin(1)/gridResolution;
    y = round(end_effector(2, i)/gridResolution) - gridmin(2)/gridResolution;
    z = round(end_effector(3, i)/gridResolution) - gridmin(3)/gridResolution;

    capability_value(x,y,z) = capability_value(x,y,z) + 1;
    i
end

%%

normalized_cap = capability_value / max(max(max(capability_value)));
counter = 1;

for i = 1:gridExtent(1)/gridResolution
    for j = 1:gridExtent(2)/gridResolution
        for k = 1:gridExtent(3)/gridResolution
            
            if normalized_cap(i,j,k) ~= 0
            xx(counter) = i*gridResolution + gridmin(1);
            yy(counter) = j*gridResolution + gridmin(2);
            zz(counter) = k*gridResolution + gridmin(3);
            cap(counter) = normalized_cap(i,j,k);
            counter = counter + 1
            end
        end
    end
end




% Create 3D scatter plot
figure(1)
scatter3(xx(1:5:end), yy(1:5:end), zz(1:5:end), 10, cap(1:5:end), 'filled');
colorbar; % Add a color bar to show the mapping of quality values to colors
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Scatter with Capability and Target Areas');
hold on

% Create target surfaces of ground and basket
X = [20 20 270 270 20 20 270 270];
Y = [125 -125 -125 125 125 -125 -125 125];
Z = [-10 -10 -10 -10 10 10 10 10];
patch('Vertices', [X' Y' Z'], 'Faces', [1 2 3 4; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1; 5 6 7 8], 'FaceColor', [0.5 0.5 0.5]);
hold on

X = [-60 -60 -120 -120     -60 -60 -120 -120];
Y = [30 -30 -30 30          30 -30  -30   30];
Z = [217 217 217 217         237 237 237 237];
patch('Vertices', [X' Y' Z'], 'Faces', [1 2 3 4; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1; 5 6 7 8], 'FaceColor', [0.5 0.5 0.5]);

% view X-Y with z = 0 (ground)
counter = 1;
clear xxx
clear yyy
clear cap0
for i = 1:length(cap)
    if zz(i) >= -20 && zz(i) <= 20
        xxx(counter) = xx(i);
        yyy(counter) = yy(i);
        cap0(counter) = cap(i);
        counter = counter + 1;
    end
end

figure(2)
scatter(xxx,yyy, 30, cap0, 'filled');
colorbar;
xlabel('X');
ylabel('Y');
title('2D Scatter on Ground level (z=0)');
hold on
X = [20 20 270 270 20 20 270 270];
Y = [125 -125 -125 125 125 -125 -125 125];
p = patch(X,Y, [0.5 0.5 0.5]);
p.FaceAlpha = 0.4;

% view X-Y with z = 227 (basket)
counter = 1;
clear xxx
clear yyy
clear cap0
for i = 1:length(cap)
    if zz(i) >= 200 && zz(i) <= 240
        xxx(counter) = xx(i);
        yyy(counter) = yy(i);
        cap0(counter) = cap(i);
        counter = counter + 1;
    end
end

figure(3)
scatter(xxx,yyy, 30, cap0, 'filled');
colorbar;
xlabel('X');
ylabel('Y');
title('2D Scatter on Basket level (z=227 mm)');
hold on
X = [-60 -60 -120 -120     -60 -60 -120 -120];
Y = [30 -30 -30 30          30 -30  -30   30];
p = patch(X,Y, [0.5 0.5 0.5]);
p.FaceAlpha = 0.4;

% view Y-Z with y = 0 (basket)
counter = 1;
clear zzz
clear xxx
clear cap0
for i = 1:length(cap)
    if yy(i) >= -20 && yy(i) <= 20
        zzz(counter) = zz(i);
        xxx(counter) = xx(i);
        cap0(counter) = cap(i);
        counter = counter + 1;
    end
end

figure(4)
scatter(xxx,zzz, 30, cap0, 'filled');
colorbar;
xlabel('X');
ylabel('Z');
title('2D Scatter on y = 0');
hold on
X = [ 20 270 270 20];
Z = [10 10 -10 -10];
p = patch(X,Z, [0.5 0.5 0.5]);
p.FaceAlpha = 0.4;
hold on
X = [-60 -60 -120 -120];
Z = [230 220 220 230];
p = patch(X,Z, [0.5 0.5 0.5]);
p.FaceAlpha = 0.4;

