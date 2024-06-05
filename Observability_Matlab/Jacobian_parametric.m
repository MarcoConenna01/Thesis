function [J] = Jacobian_parametric(theta)

    eps = 0.00001;
    
    % angles theta as input
    % d
    d = [76 0 136 0 129 0];
    % extra translations of joints 2 and 4
    dd = [70 70];
    % a
    a = [0 0 0 0 0 0];
    % alpha 
    alpha = pi/2*[1 -1 1 -1 1 0];
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

    % center of mass position wrt. same joint position, x y z
    cm = [ 0 0 30; ... %1 DOF
    0 0 20; ...
    0 0 30; ...
    0 0 30; ...
    0 0 30; ...
    0 0 50]; % gripper

    % mass of each link
    m = [100 100 100 100 100 200];

    % df/dx alpha
    df_dx_alpha1 = (End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1+eps -1 1 -1 1 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1-eps -1 1 -1 1 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    df_dx_alpha2 = (End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1+eps 1 -1 1 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1-eps 1 -1 1 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    df_dx_alpha3 = (End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1 1+eps -1 1 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1 1-eps -1 1 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    df_dx_alpha4 = (End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1 1 -1+eps 1 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1 1 -1-eps 1 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    df_dx_alpha5 = (End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1 1 -1 1+eps 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1 1 -1 1-eps 0], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    df_dx_alpha6 = (End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1 1 -1 1 0+eps], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, pi/2*[1 -1 1 -1 1 0-eps], x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(pi*eps);
    
    % df/dx a
    df_dx_a1 = (End_effector_pos(m, cm, theta, d, dd, [eps 0 0 0 0 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, [-eps 0 0 0 0 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_a2 = (End_effector_pos(m, cm, theta, d, dd, [0 eps 0 0 0 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, [0 -eps 0 0 0 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_a3 = (End_effector_pos(m, cm, theta, d, dd, [0 0 eps 0 0 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, [0 0 -eps 0 0 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_a4 = (End_effector_pos(m, cm, theta, d, dd, [0 0 0 eps 0 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, [0 0 0 -eps 0 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_a5 = (End_effector_pos(m, cm, theta, d, dd, [0 0 0 0 eps 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, [0 0 0 0 -eps 0], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_a6 = (End_effector_pos(m, cm, theta, d, dd, [0 0 0 0 0 eps], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, [0 0 0 0 0 -eps], alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx theta (TO DO: APPLY WRENCH)
    df_dx_theta1 = (End_effector_pos(m, cm, theta + [eps 0 0 0 0 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta - [eps 0 0 0 0 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_theta2 = (End_effector_pos(m, cm, theta + [0 eps 0 0 0 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta - [0 eps 0 0 0 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_theta3 = (End_effector_pos(m, cm, theta + [0 0 eps 0 0 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta - [0 0 eps 0 0 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_theta4 = (End_effector_pos(m, cm, theta + [0 0 0 eps 0 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta - [0 0 0 eps 0 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_theta5 = (End_effector_pos(m, cm, theta + [0 0 0 0 eps 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta - [0 0 0 0 eps 0], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_theta6 = (End_effector_pos(m, cm, theta + [0 0 0 0 0 eps], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta - [0 0 0 0 0 eps], d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx d
    df_dx_d1 = (End_effector_pos(m, cm, theta, d + [eps 0 0 0 0 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d - [eps 0 0 0 0 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_d2 = (End_effector_pos(m, cm, theta, d + [0 eps 0 0 0 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d - [0 eps 0 0 0 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_d3 = (End_effector_pos(m, cm, theta, d + [0 0 eps 0 0 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d - [0 0 eps 0 0 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_d4 = (End_effector_pos(m, cm, theta, d + [0 0 0 eps 0 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d - [0 0 0 eps 0 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_d5 = (End_effector_pos(m, cm, theta, d + [0 0 0 0 eps 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d - [0 0 0 0 eps 0], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_d6 = (End_effector_pos(m, cm, theta, d + [0 0 0 0 0 eps], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d - [0 0 0 0 0 eps], dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx position base
    df_dx_bx  = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base + eps, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base - eps, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_by  = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base + eps, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base - eps, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_bz   = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base + eps, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base - eps, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx orientation base (TODO: APPLY WRENCH)
    df_dx_balpha  = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha + eps, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha - eps, angle_base_beta))/(2*eps);
    df_dx_bbeta  = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta + eps) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta - eps))/(2*eps); 
    
    % df/dx position end_effector
    df_dx_Tx  = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper + eps, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper - eps, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_Ty  = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper + eps, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper - eps, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_Tz  = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper + eps, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper - eps, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    
    % df/dx orientation end_effector
    df_dx_Talpha = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha + eps, angle_gripper_beta, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha - eps, angle_gripper_beta, angle_base_alpha, angle_base_beta))/(2*eps);
    df_dx_Tbeta  = (End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta + eps, angle_base_alpha, angle_base_beta) - End_effector_pos(m, cm, theta, d, dd, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta - eps, angle_base_alpha, angle_base_beta))/(2*eps);
    
    J = [df_dx_alpha1, df_dx_alpha2, df_dx_alpha3, df_dx_alpha4, df_dx_alpha5, df_dx_alpha6, ...
         df_dx_a1, df_dx_a2, df_dx_a3, df_dx_a4, df_dx_a5, df_dx_a6, ...
         df_dx_theta1, df_dx_theta2, df_dx_theta3, df_dx_theta4, df_dx_theta5, df_dx_theta6, ...
         df_dx_d1, df_dx_d2, df_dx_d3, df_dx_d4, df_dx_d5, df_dx_d6, ...
         df_dx_bx, df_dx_by, df_dx_bz, ...
         df_dx_balpha, df_dx_bbeta, ...
         df_dx_Tx, df_dx_Ty, df_dx_Tz, ...
         df_dx_Talpha, df_dx_Tbeta];

end
