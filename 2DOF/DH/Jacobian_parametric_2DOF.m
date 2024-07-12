function [J] = Jacobian_parametric_2DOF(theta, v, eps2, option)
    
    n_parameters = 27;
    %eps_E = eps2;
    eps_E = 6e-11;
    eps = eps2;

    par = zeros(6,n_parameters);
    for i = 1:n_parameters
        if i > 6 && i < 10 % if it's elastic coefficients
            eps = eps_E;
        elseif i >= 10
            eps = eps2;
        end
        v(i) = v(i) + eps;
        a = FK_2DOF(theta, v);
        v(i) = v(i) - 2*eps;
        b = FK_2DOF(theta, v);
        par(:,i) = (a-b)/(2*eps);
        v(i) = v(i) + eps;
    end

    df_dx_alpha1 = par(:,1);
    df_dx_alpha2 = par(:,2);
    df_dx_a1 = par(:,3);
    df_dx_a2 = par(:,4);
    df_dx_theta1 = par(:,5);
    df_dx_theta2 = par(:,6);
    df_dx_k0 = par(:,7);
    df_dx_k1 = par(:,8);
    df_dx_k2 = par(:,9);
    df_dx_d1 = par(:,10);
    df_dx_d2 = par(:,11);
    df_dx_bx = par(:,12);
    df_dx_by = par(:,13);
    df_dx_bz = par(:,14);
    df_dx_balpha = par(:,15);
    df_dx_bbeta = par(:,16);
    df_dx_Tx = par(:,17);
    df_dx_Ty = par(:,18);
    df_dx_Tz = par(:,19);
    df_dx_Talpha = par(:,20);
    df_dx_Tbeta = par(:,21);
    df_dx_cm1x = par(:,22);
    df_dx_cm1y = par(:,23);
    df_dx_cm1z = par(:,24);
    df_dx_cm2x = par(:,25);
    df_dx_cm2y = par(:,26);
    df_dx_cm2z = par(:,27);

    if option == 1
    % OPTION 1: All the parameters: 27
    J = [df_dx_alpha1, df_dx_alpha2, ...
         df_dx_a1, df_dx_a2, ...
         df_dx_theta1, df_dx_theta2, ...
         df_dx_k0, df_dx_k1, df_dx_k2, ...
         df_dx_d1, df_dx_d2, ...
         df_dx_bx, df_dx_by, df_dx_bz, ...
         df_dx_balpha, df_dx_bbeta, ...
         df_dx_Tx, df_dx_Ty, df_dx_Tz, ...
         df_dx_Talpha, df_dx_Tbeta ...
         df_dx_cm1x, df_dx_cm1y, df_dx_cm1z ...
         df_dx_cm2x, df_dx_cm2y, df_dx_cm2z];

    elseif option == 2
    % OPTION 2: Only the parameters that make the matrix full rank: 15    
    J = [df_dx_alpha1, df_dx_alpha2, ...
         df_dx_a1, ...
         df_dx_theta1, df_dx_theta2, ...
         df_dx_k0, df_dx_k1, df_dx_k2, ...
         df_dx_bx, df_dx_by, df_dx_bz, ...
         df_dx_balpha, ...
         df_dx_Tz ...
         df_dx_cm2x, df_dx_cm2y];

    elseif option == 3
    % OPTION 3: Only the parameters that make the matrix full rank without cm: 13   
    J = [df_dx_alpha1, df_dx_alpha2, ...
         df_dx_a1, ...
         df_dx_theta1, df_dx_theta2, ...
         df_dx_k0, df_dx_k1, df_dx_k2, ...
         df_dx_bx, df_dx_by, df_dx_bz, ...
         df_dx_balpha, ...
         df_dx_Tz];

    elseif option == 4
    % OPTION 4: 2 without the elastic parameters
    J = [df_dx_alpha1, df_dx_alpha2, ...
         df_dx_a1, ...
         df_dx_theta1, df_dx_theta2, ...
         df_dx_bx, df_dx_by, df_dx_bz, ...
         df_dx_balpha, ...
         df_dx_Tz ...
         df_dx_cm2x, df_dx_cm2y];

    elseif option == 5
    % OPTION 5: only the elastic coefficients
    J = [df_dx_k0, df_dx_k1, df_dx_k2];

    elseif option == 6
    % OPTION 6 (TEST): only tau0
    J = df_dx_k0;
   
    end

end

