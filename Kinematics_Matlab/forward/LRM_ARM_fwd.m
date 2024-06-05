%% BASIC FORWARD KINEMATICS OF LRM ARM
clear all
close all
clc

% angles
theta = [0 0 0 0 0 0];

% d
d = [76 0 136 0 129 0];

% a
a = [0 0 0 0 0 0];

% alpha 
alpha = pi/2*[1 -1 1 -1 1 0];

% end-effector translation and rotation
A1 = [1 0 0 0; 0 1 0 129; 0 0 1 0; 0 0 0 1];
A2 = [1 0 0 0; 0 cos(pi/2) -sin(pi/2) 0; 0 sin(pi/2) cos(pi/2) 0; 0 0 0 1];
A = A1*A2;

% initial translation and rotation
A1 = [1 0 0 0; 0 1 0 0; 0 0 1 227; 0 0 0 1];
A2 = [cos(pi/3) 0 sin(pi/3) 0; 0 1 0 0; -sin(pi/3) 0 cos(pi/3) 0; 0 0 0 1];
A_base = A1*A2;

% initial configuration
j0 = [0 0 0];
j1 = [0 0 227];
j2 = j1 + 76*[1 0 1]/sqrt(2);
j3 = j2 + 136*[1 0 1]/sqrt(2);
j4 = j3 + 129*[1 0 1]/sqrt(2);
j5 = j4 + 129*[1 0 1]/sqrt(2);

x = [j0(1) j1(1) j2(1) j3(1) j4(1)];
y = [j0(2) j1(2) j2(2) j3(2) j4(2)];
z = [j0(3) j1(3) j2(3) j3(3) j4(3)];

A01 = buildHD(theta(1),alpha(1),d(1),a(1));
A12 = buildHD(theta(2),alpha(2),d(2),a(2));
A23 = buildHD(theta(3),alpha(3),d(3),a(3));
A34 = buildHD(theta(4),alpha(4),d(4),a(4));
A45 = buildHD(theta(5),alpha(5),d(5),a(5));
A56 = buildHD(theta(6),alpha(6),d(6),a(6));

pos1 = A_base*A01*[j0';1];
pos2 = A_base*A01*A12*[j0';1];
pos3 = A_base*A01*A12*A23*[j0';1];
pos4 = A_base*A01*A12*A23*A34*[j0';1];
pos5 = A_base*A01*A12*A23*A34*A45*[j0';1];
pos6 = A_base*A01*A12*A23*A34*A45*A56*[j0';1];
posgripper = A_base*A01*A12*A23*A34*A45*A56*A*[j0';1];

posx = [0 0 pos1(1) pos2(1) pos3(1) pos4(1) pos5(1) pos6(1) posgripper(1)];
posy = [0 0 pos1(2) pos2(2) pos3(2) pos4(2) pos5(2) pos6(2) posgripper(2)];
posz = [227 0 pos1(3) pos2(3) pos3(3) pos4(3) pos5(3) pos6(3) posgripper(3)];

% Plot the points as red big dots with a line connecting them
figure(1);
scatter3(posx,posy,posz, 'o', 'MarkerFaceColor', 'b','MarkerEdgeColor', 'b');
grid on;
axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Coordinates');
hold on
scatter3(x,y,z, 'o', 'MarkerFaceColor', 'r','MarkerEdgeColor', 'r');


%% test workspace

% aperture +-100 degrees = 10 pi/18
number = 15;
field = linspace(-10*pi/18,10*pi/18,number);
theta_test = field'*ones(1,6);
end_effector = zeros(3,number^6);
quality = zeros(1,number^6);
counter = 1;

for i = 1:number
    for j = 1:number
        for k = 1:number
            for l = 1:number
                for z = 1:number
                    for x = 1:number

                        A01 = buildHD(theta_test(i,1),alpha(1),d(1),a(1));
                        A12 = buildHD(theta_test(j,2),alpha(2),d(2),a(2));
                        A23 = buildHD(theta_test(k,3),alpha(3),d(3),a(3));
                        A34 = buildHD(theta_test(l,4),alpha(4),d(4),a(4));
                        A45 = buildHD(theta_test(z,5),alpha(5),d(5),a(5));
                        A56 = buildHD(theta_test(x,6),alpha(6),d(6),a(6));
                        
                        posgripper = A_base*A01*A12*A23*A34*A45*A56*A*[j0';1];

                        end_effector(1,counter) = posgripper(1);
                        end_effector(2,counter) = posgripper(2);
                        end_effector(3,counter) = posgripper(3);
                        
                        quality(counter) =  mean(pi/2 - [abs(theta_test(i,1)) abs(theta_test(j,2)) abs(theta_test(k,3)) abs(theta_test(l,4)) abs(theta_test(z,5)) abs(theta_test(x,6))])*2/pi;

                        counter = counter +1
                    end
                end
            end
        end
    end
end

% Create 3D scatter plot
% figure;
% scatter3(end_effector(1,:), end_effector(2,:), end_effector(3,:), 50, quality, 'filled');
% colorbar; % Add a color bar to show the mapping of quality values to colors
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('3D Scatter Plot with Quality Coloring');

% figure(2)
% for i=1:length(end_effector)
%     %if end_effector(3,i) < 0.5 && end_effector(3,i) > -0.5
%     plot3(end_effector(1,i),end_effector(2,i),end_effector(3,i), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'r','MarkerEdgeColor', 'r');
% 
%     hold on
%     %end
% end

% figure(3)
% for i=1:length(end_effector)
%     if end_effector(3,i) < 1 && end_effector(3,i) > -1
%         planexy(1,i) = end_effector(1,i);
%         planexy(2,i) = end_effector(2,i);
%         planexy(3,i) = quality(i);
%     end
% end
% scatter(planexy(1,:),planexy(2,:),planexy(3,:),'filled','colorvariable')
% colorbar



function [A] = buildHD(theta, alpha, d, a)
    A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);...
       sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
       0 sin(alpha) cos(alpha) d;...
       0 0 0 1];
end



