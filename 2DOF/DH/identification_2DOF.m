clc
clear all

%% IDENTIFICATION

% import estimated positions
load psi_3
ii = 5;

% import measured positions (TEST: faking a tau0 of 10^-7)
x_m = zeros(height(psi),1);
v = zeros(1,21);
v(7) = 10^-7;
for i = 1:height(psi)
    theta = psi(i,2:3);
    P = FK_2DOF(theta, v);
    x_m(3*i-2:3*i,1) = P(1:3);
end

option = 3;
x_est = zeros(height(psi),1);
v = zeros(21,1);
error = 100;
treshold = 0.0001;
eps = 10^-8;
while error > treshold

    % use current v to calculate the Identification Jacobian Matrix J
    for j = 1:20
        theta = psi(j,2:3);
        J = Jacobian_parametric_2DOF(theta,v, eps, option);
        Jacobian_total(j*3-2:j*3,:) = J(1:3,:);
    end

    % use the current v to find the estimated position for all the conf
    for i = 1:height(psi)
        theta = psi(i,2:3);
        P = FK_2DOF(theta, v);
        x_est(3*i-2:3*i,1) = P(1:3);
    end

    % position error calculation
    deltaX = x_m - x_est;
    deltav = pinv(Jacobian_total)*deltaX;

    if option == 1
        v = v + deltav;
    elseif option == 2
        v(1:3) = v(1:3) + deltav(1:3);
        v(5:9) = v(5:9) + deltav(4:8);
        v(12:16) = v(12:16) + deltav(9:13);
        v(19) = v(19) + deltav(14);
    elseif option == 3
        v(1:3) = v(1:3) + deltav(1:3);
        v(5:6) = v(5:6) + deltav(4:5);
        v(12:16) = v(12:16) + deltav(6:10);
        v(19) = v(19) + deltav(11);
    elseif option == 4
        v(7:9) = v(7:9) + deltav;
    elseif option == 5
        v(7) = v(7) + deltav;
    end

    error = norm(deltaX)
    
end

%    J = [df_dx_alpha1, df_dx_alpha2, ...
%          df_dx_a1, df_dx_a2, ...
%          df_dx_theta1, df_dx_theta2, ...
%          df_dx_tau0, df_dx_tau1, df_dx_tau2, ...
%          df_dx_d1, df_dx_d2, ...
%          df_dx_bx, df_dx_by, df_dx_bz, ...
%          df_dx_balpha, df_dx_bbeta, ...
%          df_dx_Tx, df_dx_Ty, df_dx_Tz, ...
%          df_dx_Talpha, df_dx_Tbeta];


% J = [df_dx_alpha1, df_dx_alpha2, ...
%          df_dx_a1, ...
%          df_dx_theta1, df_dx_theta2, ...
%          df_dx_tau0, df_dx_tau1, df_dx_tau2, ...
%          df_dx_bx, df_dx_by, df_dx_bz, ...
%          df_dx_balpha, df_dx_bbeta, ...
%          df_dx_Tz];







