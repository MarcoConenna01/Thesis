function [J] = Jacobian_parametric(theta, v, eps, option)

    n_parameters = 42;
    par = zeros(6,n_parameters);

    for i = 1:n_parameters
        v(i) = v(i) + eps;
        a = FK(theta, v);
        v(i) = v(i) - 2*eps;
        b = FK(theta, v);
        par(:,i) = (a-b)/(2*eps);
        v(i) = v(i) + eps;
    end

    df_dx_alpha1 = par(:,1);
    df_dx_alpha2 = par(:,2);%
    df_dx_alpha3 = par(:,3);%
    df_dx_alpha4 = par(:,4);%
    df_dx_alpha5 = par(:,5);%
    df_dx_alpha6 = par(:,6);
    df_dx_tau0 = par(:,7);%
    df_dx_tau1 = par(:,8);%
    df_dx_tau2 = par(:,9);%
    df_dx_tau3 = par(:,10);%
    df_dx_tau4 = par(:,11);%
    df_dx_tau5 = par(:,12);%
    df_dx_tau6 = par(:,13);%
    df_dx_a1 = par(:,14);
    df_dx_a2 = par(:,15);
    df_dx_a3 = par(:,16);%
    df_dx_a4 = par(:,17);%
    df_dx_a5 = par(:,18);%
    df_dx_a6 = par(:,19);
    df_dx_theta1 = par(:,20);%
    df_dx_theta2 = par(:,21);%
    df_dx_theta3 = par(:,22);%
    df_dx_theta4 = par(:,23);%
    df_dx_theta5 = par(:,24);%
    df_dx_theta6 = par(:,25);%
    df_dx_d1 = par(:,26);
    df_dx_d2 = par(:,27);
    df_dx_d3 = par(:,28);
    df_dx_d4 = par(:,29);%
    df_dx_d5 = par(:,30);%
    df_dx_d6 = par(:,31);
    df_dx_bx = par(:,32);%
    df_dx_by = par(:,33);%
    df_dx_bz = par(:,34);%
    df_dx_balpha = par(:,35);
    df_dx_bbeta = par(:,36);
    df_dx_Tx = par(:,37);
    df_dx_Ty = par(:,38);%
    df_dx_Tz = par(:,39);%
    df_dx_Talpha = par(:,40);
    df_dx_Tbeta = par(:,41);
    df_dx_dd1 = par(:,42);

    if option == 1
        J = [df_dx_alpha2, df_dx_alpha3, df_dx_alpha4, df_dx_alpha5, ...
             df_dx_tau0, df_dx_tau1, df_dx_tau2, df_dx_tau3, df_dx_tau4, df_dx_tau5, df_dx_tau6, ...
             df_dx_a3, df_dx_a4, df_dx_a5, ...
             df_dx_theta1, df_dx_theta2, df_dx_theta3, df_dx_theta4, df_dx_theta5, df_dx_theta6, ...
             df_dx_d4, df_dx_d5, ...
             df_dx_bx, df_dx_by, df_dx_bz, ...
             df_dx_Ty, df_dx_Tz ...
             df_dx_dd1];
    elseif option == 2
             J = [df_dx_alpha2, df_dx_alpha3, df_dx_alpha4, df_dx_alpha5, ...
             df_dx_a3, df_dx_a4, df_dx_a5, ...
             df_dx_theta1, df_dx_theta2, df_dx_theta3, df_dx_theta4, df_dx_theta5, df_dx_theta6, ...
             df_dx_d4, df_dx_d5, ...
             df_dx_bx, df_dx_by, df_dx_bz, ...
             df_dx_Ty, df_dx_Tz];
    elseif option == 3
         J = [df_dx_tau0, df_dx_tau1, df_dx_tau2, df_dx_tau3, df_dx_tau4, df_dx_tau5, df_dx_tau6];
    elseif option == 4
         J = [df_dx_alpha2, df_dx_alpha3, df_dx_alpha4, df_dx_alpha5, ...
             df_dx_tau1, df_dx_tau2, df_dx_tau3, df_dx_tau4, df_dx_tau5, df_dx_tau6, ...
             df_dx_a3, df_dx_a4, df_dx_a5, ...
             df_dx_theta1, df_dx_theta2, df_dx_theta3, df_dx_theta4, df_dx_theta5, df_dx_theta6, ...
             df_dx_d4, df_dx_d5, ...
             df_dx_bx, df_dx_by, df_dx_bz, ...
             df_dx_Ty, df_dx_Tz ...
             df_dx_dd1];
    elseif option == 5
        J = [df_dx_tau1, df_dx_tau2, df_dx_tau3, df_dx_tau4, df_dx_tau5, df_dx_tau6];
    end

end
