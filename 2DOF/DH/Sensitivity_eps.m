clc
clear all

%% List of possible conf-wrench combinations

n_parameters = 21;
v = zeros(1,n_parameters);
eps = 10^-10;
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

%% Sensitivity analysis of the perturbation factor epsilon
for k = 1:length(configuration)
    k
    theta = configuration(k,2:3);
    v = zeros(1, 21);
    % eps = linspace(10^-20, 10^-5, 1000);  % Reverse order for eps
    eps = logspace(log10(10^-20), log10(10^-8), 1000);
    D_norm = zeros(1, length(eps));
    D_norm_nonE = zeros(1, length(eps));
    D_norm_E = zeros(1, length(eps));
    J_norm_old = 1000;
    J_norm_nonE_old = 1000;
    J_norm_E_old = 1000;
    
    for j = 1:length(eps)
        J= Jacobian_parametric_2DOF(theta, v, eps(j));  
        J_nonelastic = [J(:,1:6) J(:,10:end)];
        J_elastic = [J(:,7:9)];
        J_norm = norm(J);
        J_norm_nonE = norm(J_nonelastic);
        J_norm_E = norm(J_elastic);
        D_norm(j) = abs(J_norm - J_norm_old);
        D_norm_nonE(j) = abs(J_norm_nonE - J_norm_nonE_old);
        D_norm_E(j) = abs(J_norm_E - J_norm_E_old);
        J_norm_old = J_norm;
        J_norm_nonE_old = J_norm_nonE;
        J_norm_E_old = J_norm_E;
        j;
    end
    
    [~, Dmin] = min(abs(D_norm - 10^-6));
    eps_target(k) = eps(Dmin);

end

% % Plotting
% plot(eps, D_norm_nonE, 'b-', 'LineWidth', 2);  % Blue solid line with increased width
% set(gca, 'XScale', 'log', 'YScale', 'log');  % Logarithmic scale for both axes
% xlabel('\epsilon', 'FontSize', 12);  % Label for x-axis
% ylabel('||J||', 'FontSize', 12);  % Label for y-axis
% title('Norm of Jacobian vs. Parameter \epsilon (elastic)', 'FontSize', 14);  % Title of the plot
% grid on;  % Display grid lines
% 
% % Reverse the direction of x-axis
% set(gca, 'XDir', 'reverse');
