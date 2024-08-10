clc
clearvars

%% Computations
v = zeros(42,1);
%theta = zeros(1,6);
theta = pi/4*ones(1,6);
%theta = pi/2*ones(1,6);


eps = logspace(log10(10^-15), log10(5*10^-4), 1000);
n_eps = length(eps);
J_norm_all = zeros(1,n_eps);
J_norm_kin = zeros(1,n_eps);
J_norm_el = zeros(1,n_eps);

for i = 1:n_eps
    i

    option = 1;
    J = Jacobian_parametric(theta, v, eps(i), option);
    J_norm_all(i) = norm(J);

    option = 2;
    J = Jacobian_parametric(theta, v, eps(i), option);
    J_norm_kin(i) = norm(J);

    option = 3;
    J = Jacobian_parametric(theta, v, eps(i), option);
    J_norm_el(i) = norm(J);
   
end

J_norm_kin_normalized = J_norm_kin/J_norm_kin(900);
J_norm_el_normalized = J_norm_el/J_norm_el(700);

%% Plots

figure(1)
plot(eps, J_norm_el, 'b-', 'LineWidth', 1);
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('\epsilon', 'FontSize', 12); 
ylabel('||J||', 'FontSize', 12);
title('Norm of Jacobian vs. Step-Size \epsilon [Elastic]', 'FontSize', 14);
grid on;
set(gca, 'XDir', 'reverse');
xlim([10^-15 5*10^-4])

figure(2)
plot(eps, J_norm_kin, 'r-', 'LineWidth', 1);
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('\epsilon', 'FontSize', 12); 
ylabel('||J||', 'FontSize', 12);
title('Norm of Jacobian vs. Step-size \epsilon [Kinematic]', 'FontSize', 14);
grid on;
set(gca, 'XDir', 'reverse');
xlim([10^-15 5*10^-4])

figure(3)
plot(eps, J_norm_kin_normalized, 'r-', 'LineWidth', 1);
hold on
plot(eps, J_norm_el_normalized, 'b-', 'LineWidth', 1);
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('\epsilon', 'FontSize', 12); 
ylabel('||J||', 'FontSize', 12);
title('Norm of Jacobian vs. Step-size \epsilon [Normalized]', 'FontSize', 14);
grid on;
set(gca, 'XDir', 'reverse');
xlim([10^-15 5*10^-4])
legend('Kinematic', 'Elastic')