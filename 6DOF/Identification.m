clc
clearvars

%% IDENTIFICATION

% import chosen configurations
load psi_2
load goodconf_corrected
goodconf = goodconf_corrected;
option = 2;

% import measured positions 
load measured_rotated.mat
position_measured = -position_measured_rotated;
position_measured(:,3) = -position_measured(:,3); % to correct for base frame placed backwards
g_m = zeros(height(psi),2);
for i = 1:height(psi)
    for j = 1:height(goodconf)
        if psi(i,1) == goodconf(j,1)
            x_m(3*i-2:3*i,1) = position_measured(j,:);
            break
        end
    end
end

x_est = zeros(height(psi),1);
v = zeros(42,1);
error = 100;
error2 = 0;
delta_old = 1000;
treshold = 0.0001;
eps = 10^-8;
counter = 1;
while error > eps

    % use current v to calculate the Identification Jacobian Matrix J
    for j = 1:height(psi)
        theta = psi(j,2:7);
        J = Jacobian_parametric(theta,v, eps, option);
        Jacobian_total(j*3-2:j*3,:) = J(1:3,:);
    end

    % use the current v to find the estimated position for all the conf
    for i = 1:height(psi)
        theta = psi(i,2:7);
        P = FK(theta, v);
        x_est(3*i-2:3*i,1) = P(1:3);
    end

    % position error calculation
    deltaX = x_m - x_est;
    deltav = pinv(Jacobian_total)*deltaX;

    if option == 1
        v(2:5) = v(2:5) + deltav(1:4);
        v(7:13) = v(7:13) + deltav(5:11);
        v(16:18) = v(16:18) + deltav(12:14);
        v(20:25) = v(20:25) + deltav(15:20);
        v(29:30) = v(29:30) + deltav(21:22);
        v(32:34) = v(32:34) + deltav(23:25);
        v(38:39) = v(38:39) + deltav(26:27);
        v(42) = v(42) + deltav(28);
    elseif option ==2
        v(2:5) = v(2:5) + deltav(1:4);
        v(16:18) = v(16:18) + deltav(5:7);
        v(20:25) = v(20:25) + deltav(8:13);
        v(29:30) = v(29:30) + deltav(14:15);
        v(32:34) = v(32:34) + deltav(16:18);
        v(38:39) = v(38:39) + deltav(19:20);
    elseif option == 3
        v(7) = v(7) + deltav;
    elseif option == 4
        v(7:13) = v(7:13) + deltav;
    end

    error(counter) = norm(deltaX)
    counter = counter + 1;
    error2 = abs(norm(deltaX)/delta_old);
    delta_old = norm(deltaX);
    
end

fprintf('The norm of the error is: %f mm\n', error);