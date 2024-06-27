function [J] = Jacobian_parametric_2DOF(theta)

    eps = 0.0001;

    % d
    d = [46.905 0];
    % a
    a = [0 0];
    % alpha 
    alpha = pi/2*[1 0];
    %gripper and base
    angle_gripper_alpha = -pi/2;
    angle_gripper_beta = 0;
    x_gripper = 0;
    y_gripper = 0;
    z_gripper = 103.505;
    angle_base_alpha = 0;
    angle_base_beta = pi/3;
    %angle_base_beta = 0;
    x_base = 0;
    y_base = 0;
    z_base = 227;

    % center of mass position wrt. same joint position, x y z
    cm = [ 0 0 45; ... %1 DOF
    0 0 65; ... %2 DOF
    ]; 

    % mass of each link
    m = [123.4 177];

    % elastic coefficients 
    tau = [0 0 0];

    % df/dx alpha
    df_dx_alpha1 = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha + [eps 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha - [eps 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    df_dx_alpha2 = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha + [0 eps], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha - [0 eps], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    
    % df/dx a
    df_dx_a1 = (End_effector_pos_2DOF(m, cm, theta, d, [eps 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, [-eps 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_a2 = (End_effector_pos_2DOF(m, cm, theta, d, [0 eps], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, [0 -eps], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx theta 
    df_dx_theta1 = (End_effector_pos_2DOF(m, cm, theta + [eps 0], d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta - [eps 0], d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_theta2 = (End_effector_pos_2DOF(m, cm, theta + [0 eps], d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta - [0 eps], d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dc elastic coefficients tau
    df_dx_tau1 = (End_effector_pos_2DOF_wrench([0 10^-10 0], m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(10^-10);
    df_dx_tau2 = (End_effector_pos_2DOF_wrench([0 0 10^-10], m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(10^-10);

    % df/dx d
    df_dx_d1 = (End_effector_pos_2DOF(m, cm, theta, d + [eps 0], a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d - [eps 0], a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_d2 = (End_effector_pos_2DOF(m, cm, theta, d + [0 eps], a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d - [0 eps], a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx position base
    df_dx_bx  = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base + eps, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base - eps, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_by  = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base + eps, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base - eps, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_bz   = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base + eps, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base - eps, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx orientation base
    df_dx_balpha  = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha + eps, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha - eps, angle_base_beta))/(2*eps);
    df_dx_bbeta  = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta + eps) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta - eps))/(2*eps); 
    
    % df/dx elastic coefficient of base structure
    df_dx_tau0 = (End_effector_pos_2DOF_wrench([10^-10 0 0], m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(10^-10); 

    % df/dx position end_effector
    df_dx_Tx  = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper + eps, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper - eps, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_Ty  = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper + eps, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper - eps, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_Tz  = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper + eps, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper - eps, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx orientation end_effector
    df_dx_Talpha = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha + eps, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha - eps, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_Tbeta  = (End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta + eps, angle_base_alpha, angle_base_beta) - End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta - eps, angle_base_alpha, angle_base_beta))/(2*eps);
    
    %% OPTION 1: All the parameters: 21

    J = [df_dx_alpha1, df_dx_alpha2, ...
         df_dx_a1, df_dx_a2, ...
         df_dx_theta1, df_dx_theta2, ...
         df_dx_tau0, df_dx_tau1, df_dx_tau2, ...
         df_dx_d1, df_dx_d2, ...
         df_dx_bx, df_dx_by, df_dx_bz, ...
         df_dx_balpha, df_dx_bbeta, ...
         df_dx_Tx, df_dx_Ty, df_dx_Tz, ...
         df_dx_Talpha, df_dx_Tbeta];

    %% OPTION 2: Only the parameters that make the matrix full rank: 14 
        
%     J = [df_dx_alpha1, df_dx_alpha2, ...
%          df_dx_a1, ...
%          df_dx_theta1, ...
%          df_dx_tau0, df_dx_tau1, df_dx_tau2, ...
%          df_dx_bx, df_dx_by, df_dx_bz, ...
%          df_dx_balpha, df_dx_bbeta, ...
%          df_dx_Tx, df_dx_Tz];

    %% OPTION 3: 2 without the elastic parameters

%     J = [df_dx_alpha1, df_dx_alpha2, ...
%          df_dx_a1, ...
%          df_dx_theta1, ...
%          df_dx_bx, df_dx_by, df_dx_bz, ...
%          df_dx_balpha, df_dx_bbeta, ...
%          df_dx_Tx, df_dx_Tz];

   %% OPTION 4: only the elastic coefficients

%     J = [df_dx_tau0, df_dx_tau1, df_dx_tau2];

end

