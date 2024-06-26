function [P] = End_effector_pos_2DOF_wrench(tau, m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta)

    error = 100;
    [P, t] = End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta);

    while error > 0.000000001
        P_0 = P;
        theta_new = theta + tau(1:2).*t(1:2);
        angle_base_beta_new = angle_base_beta - tau(3)*t(3);
        [P, t] = End_effector_pos_2DOF(m, cm, theta_new, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta_new);
        error = norm(abs(P-P_0));
    end


end

