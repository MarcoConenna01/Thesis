clc;
clear all;
close all;

% Inputs ---------------------------------------------------------------
q = [0 0 82.012; 
     40.6209215645091 0 105.464500000000]; % Joint positions at zero configuration
theta = [pi/2 pi/2]; % Joint angles
w = [0.8660 0 0.5000; 
     0 -1 0]; % Joint actuation velocities (twist vectors)

T0 = [0.5 0 0.866025403784439 130.258880983217; ...
      0 1 0 0; ...
      -0.866025403784439 0 0.500 157.217; ...
      0 0 0 1]; % End-effector transformation matrix at zero configuration

% Calculate Transformation Matrix --------------------------------------
T = POE(theta, q, w);
Tf = T * T0;

% Extract Rotation Matrix and Position Vector --------------------------
Rt = Tf(1:3, 1:3);
p = Tf(1:3, 4);

%% Plotting Joint Positions --------------------------------------------
figure;
hold on;
grid on;
view(3); % Set 3D view

% Plot base frame
plot3(0, 0, 0, 'ko', 'MarkerSize', 8, 'LineWidth', 2); % Base origin

% Plot joint positions
for i = 1:size(q, 1)
    plot3(q(i, 1), q(i, 2), q(i, 3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    text(q(i, 1), q(i, 2), q(i, 3), sprintf('Joint %d', i), 'FontSize', 12);
end

% Plot end-effector position
plot3(p(1), p(2), p(3), 'go', 'MarkerSize', 12, 'LineWidth', 2);
text(p(1), p(2), p(3), 'End-Effector', 'FontSize', 12);

% Plot twist vectors
scale = 20; % Scaling factor for twist vectors
for i = 1:size(q, 1)
    quiver3(q(i,1), q(i,2), q(i,3), w(i,1)*scale, w(i,2)*scale, w(i,3)*scale, 'b', 'LineWidth', 1.5);
    text(q(i,1) + w(i,1)*scale, q(i,2) + w(i,2)*scale, q(i,3) + w(i,3)*scale, sprintf('Twist %d', i), 'Color', 'blue', 'FontSize', 10);
end

% Plot axes labels
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Joint Positions, End-Effector, and Twist Vectors');

% Set equal aspect ratio
axis equal;

% Show legend
legend('Base', 'Joint Positions', 'End-Effector', 'Twist Vectors', 'Location', 'Best');

% Show plot
hold off;

%% Utility function ----------------------------------------------------
function T = POE(theta, q, w)
    T = eye(4);
    n = length(theta);

    for ii = n:-1:1
        w_hat = [0       -w(ii,3)  w(ii,2); ...
                 w(ii,3)  0        -w(ii,1); ...
                 -w(ii,2) w(ii,1)  0];        
        e_w_hat = eye(3) + w_hat*sin(theta(ii)) + w_hat*w_hat*(1-cos(theta(ii)));

        v = -cross(w(ii,:), q(ii,:));

        t = (eye(3)*theta(ii) + (1-cos(theta(ii)))*w_hat + ...
             (theta(ii)-sin(theta(ii)))*w_hat*w_hat) * v';

        e_zai = [e_w_hat t; 0 0 0 1];
    
        T = e_zai * T;
    end
end