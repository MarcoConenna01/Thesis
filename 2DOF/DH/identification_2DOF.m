clc
clearvars

%% IDENTIFICATION

% import chosen configurations
load psi_5.mat
option = 5;

% import measured positions 
load measured_rotated.mat
position_measured = -position_measured_rotated;
position_measured(:,3) = -position_measured(:,3); % to correct for arm placed 180 degrees wrt PEL frame
g_m = zeros(height(psi),2);
for i = 1:height(psi)
    x_m(3*i-2:3*i,1) = position_measured(psi(i,1),:);
end

x_est = zeros(height(psi),1);
v = zeros(27,1);
error = 100;
error2 = 0;
delta_old = 100;
treshold = 0.0001;
eps = 10^-8;
while error2 < 1 - 10^-12

    % use current v to calculate the Identification Jacobian Matrix J
    for j = 1:height(psi)
        theta = psi(j,2:3);
        gravity = g_m(j,:);
        J = Jacobian_parametric_2DOF(theta,v, eps, option);
        Jacobian_total(j*3-2:j*3,:) = J(1:3,:);
    end

    % use the current v to find the estimated position for all the conf
    for i = 1:height(psi)
        theta = psi(i,2:3);
        gravity = g_m(i,:);
        P = FK_2DOF(theta, v);
        x_est(3*i-2:3*i,1) = P(1:3);
    end

    % position error calculation
    deltaX = x_m - x_est;
    deltav = pinv(Jacobian_total,0.1)*deltaX;

    if option == 1
        v = v + deltav;
    elseif option == 2
        v(1:3) = v(1:3) + deltav(1:3);
        v(5:9) = v(5:9) + deltav(4:8);
        v(12:15) = v(12:15) + deltav(9:12);
        v(19) = v(19) + deltav(13);
        v(25) = v(25) + deltav(14); 
        v(26) = v(26) + deltav(15);
    elseif option == 3
        v(1:3) = v(1:3) + deltav(1:3);
        v(5:9) = v(5:9) + deltav(4:8);
        v(12:15) = v(12:15) + deltav(9:12);
        v(19) = v(19) + deltav(13);
    elseif option == 4
        v(1:3) = v(1:3) + deltav(1:3);
        v(5:6) = v(5:6) + deltav(4:5);
        v(12:15) = v(12:15) + deltav(6:9);
        v(19) = v(19) + deltav(10);
        v(25) = v(25) + deltav(11); 
        v(26) = v(26) + deltav(12); 
    elseif option == 5
        v(7:9) = v(7:9) + deltav;
    elseif option == 6
        v(7) = v(7) + deltav;
    end

    error = norm(deltaX)
    error2 = abs(norm(deltaX)/delta_old);
    delta_old = norm(deltaX);
    
end

fprintf('The norm of the error is: %f mm\n', error);