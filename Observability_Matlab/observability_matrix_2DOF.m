clc
clear all
close all

%% List of possible conf-wrench combinations (TODO: ADD WRENCH)

n_theta = 10;
n_configuration = n_theta^2;
field = linspace(-10*pi/18,10*pi/18,n_theta);
counter = 0;
configuration = zeros(n_configuration,3);

for i = 1:n_theta
    for j = 1:n_theta    
        counter = counter +1;
        configuration(counter,:) = [counter, field(i), field(j)];
    end
end

counter = 0;
flag = 0;
n = 50;
psi = configuration(1:n,:);
O = zeros(1, n_configuration);
added = 0; % pointer of added conf
deleted = 0; % pointer of deleted conf

while flag == 0

    for i = 1:(n_configuration) % step b
        %i
        if ~ismember(configuration(i, 1), psi(:,1)) 
            psi(n+1,:) = configuration(i, :);
            for j = 1:(n+1)    
                % Observability Matrix
                Jacobian_total(j*3-2:j*3,:) = Jacobian_par(psi(j,2:3));
            end
            % calculate index
            S = svd(Jacobian_total);
            O(i) = (prod(S))^(1/length(S))/(sqrt(n+1));
            clear Jacobian_total
        end
    end

    % check which gave the best O and add it to the conf, then do the same
    % thing deleting one
    [~, added] = max(O);
    psi(n+1, :) = [configuration(added, :)];
    O = zeros(1, n_configuration);
   
    for i = 1:(n+1) % step c
        %i
        psi_deleted = psi;
        psi_deleted(i,:) = [];
        for j = 1:(n)

            % Observability Matrix
            Jacobian_total(j*3-2:j*3,:) = Jacobian_par(psi_deleted(j,2:3));
        end

            % calculate index
            S = svd(Jacobian_total);
            O(i) = (prod(S))^(1/length(S))/(sqrt(n));
            clear Jacobian_total
       
    end
    
    [~, deleted] = max(O);

     % exit condition, deleted == added
    if psi(deleted,1) == added
        flag = 1;
    end

    psi(deleted, :) = [];
    O = zeros(1, n_configuration);
    counter = counter + 1
end


function [J] = Jacobian_par(theta)

    eps = 0.00001;
    % angles as input
    % d
    d = [76 0];
    % a
    a = [0 0];
    % alpha 
    alpha = pi/2*[1 0];
    %gripper and base
    angle_gripper_alpha = -pi/2;
    angle_gripper_beta = 0;
    x_gripper = 0;
    y_gripper = 0;
    z_gripper = 129;
    angle_base_alpha = 0;
    angle_base_beta = pi/3;
    x_base = 0;
    y_base = 0;
    z_base = 227;

    % df/dx alpha
    df_dx_alpha1 = (End_effector_pos(theta, d, a, pi/2*[1+eps 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, pi/2*[1-eps 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    df_dx_alpha2 = (End_effector_pos(theta, d, a, pi/2*[1 0+eps], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, pi/2*[1 -eps], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    
    % df/dx a
    df_dx_a1 = (End_effector_pos(theta, d, [eps 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, [-eps 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_a2 = (End_effector_pos(theta, d, [0 eps], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, [0 -eps], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx theta (TO DO: APPLY WRENCH)
    df_dx_theta1 = (End_effector_pos(theta + [eps 0], d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta - [eps 0], d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_theta2 = (End_effector_pos(theta + [0 eps], d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta - [0 eps], d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx d
    df_dx_d1 = (End_effector_pos(theta, d + [eps 0], a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d - [eps 0], a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_d2 = (End_effector_pos(theta, d + [0 eps], a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d - [0 eps], a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx position base
    df_dx_bx  = (End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base + eps, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base - eps, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_by  = (End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base + eps, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base - eps, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_bz   = (End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base + eps, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base - eps, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx orientation base (TODO: APPLY WRENCH)
    df_dx_balpha  = (End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha + eps, angle_base_beta) - End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha - eps, angle_base_beta))/(2*eps);
    df_dx_bbeta  = (End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta + eps) - End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta - eps))/(2*eps); 
    
    % df/dx position end_effector
    df_dx_Tx  = (End_effector_pos(theta, d, a, alpha, x_gripper + eps, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, alpha, x_gripper - eps, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_Ty  = (End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper + eps, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper - eps, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_Tz  = (End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper + eps, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper - eps, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx orientation end_effector
    df_dx_Talpha = (End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha + eps, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha - eps, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_Tbeta  = (End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta + eps, angle_base_alpha, angle_base_beta) - End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta - eps, angle_base_alpha, angle_base_beta))/(2*eps);
    
    J = [df_dx_alpha1, df_dx_alpha2, ...
         df_dx_a1, df_dx_a2, ...
         df_dx_theta1, df_dx_theta2, ...
         df_dx_d1, df_dx_d2, ...
         df_dx_bx, df_dx_by, df_dx_bz, ...
         df_dx_balpha, df_dx_bbeta, ...
         df_dx_Tx, df_dx_Ty, df_dx_Tz, ...
         df_dx_Talpha, df_dx_Tbeta];

end


function [P] = End_effector_pos(theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta)

    % end-effector translation and rotation
    A_transl = [1 0 0 x_gripper; 0 1 0 y_gripper; 0 0 1 z_gripper; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_gripper_alpha) -sin(angle_gripper_alpha) 0; 0 sin(angle_gripper_alpha) cos(angle_gripper_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_gripper_beta) 0 sin(angle_gripper_beta) 0; 0 1 0 0; -sin(angle_gripper_beta) 0 cos(angle_gripper_beta) 0; 0 0 0 1];
    A = Aalpha*Abeta*A_transl;
    
    % base translation and rotation
    A_transl = [1 0 0 x_base; 0 1 0 y_base; 0 0 1 z_base; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_base_alpha) -sin(angle_base_alpha) 0; 0 sin(angle_base_alpha) cos(angle_base_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_base_beta) 0 sin(angle_base_beta) 0; 0 1 0 0; -sin(angle_base_beta) 0 cos(angle_base_beta) 0; 0 0 0 1];
    A_base = A_transl*Aalpha*Abeta;
    
    % Compute transformation matrices
    A01 = buildHD(theta(1), alpha(1), d(1), a(1));
    A12 = buildHD(theta(2), alpha(2), d(2), a(2));
    
    % Compute positions of each joint
    T01 = A_base * A01;
    T02 = T01 * A12;
    Tgripper = T02 * A;
    P = Tgripper(1:3, 4);

end

% Define the buildHD function
function [A] = buildHD(theta, alpha, d, a)
    A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);...
       sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
       0 sin(alpha) cos(alpha) d;...
       0 0 0 1];
end
