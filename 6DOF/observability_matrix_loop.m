clc
clearvars
close all
load LRM_ARM_data/measured_rotated.mat

% List of possible conf-wrench combinations
n_parameters = 42;
v = zeros(1,n_parameters);
eps = 10^-9;
option = 1;

load LRM_ARM_data/goodconf_corrected.mat
configuration = goodconf_corrected;
n_configuration = height(configuration);

% Observability Matrix
counter = 0;
flag = 0;
flag2 = 0;
n = 50;
psi = configuration(1:n,:);
O = zeros(1, n_configuration);
added = 0; % pointer of added conf
deleted = 0; % pointer of deleted conf

while flag == 0
    tic
    for i = 1:(n_configuration) % step b
        i;
        if ~ismember(configuration(i, 1), psi(:,1)) 
            psi(n+1,:) = configuration(i, :);
            for j = 1:(n+1)    
                % Observability Matrix
                J = Jacobian_parametric(psi(j,2:end), v, eps, option);
                Jacobian_total(j*3-2:j*3,:) = J(1:3,:);
            end
            % calculate index
            S = svd(Jacobian_total);
            O(i) = (prod(S))^(1/length(S))/(sqrt(n+1));
            clear Jacobian_total
        end
    end

    % check which gave the best O (observability index) and add it to the conf, then do the same
    % thing deleting one
    % check if max has coefficients positive
    psi2 = psi;
    [~, added] = max(O);
    while flag2 == 0
    psi2(n+1, :) = [configuration(added, :)];
    v_test = Identification_loop(option, psi2, goodconf_corrected, position_measured_rotated);
    if any(v_test(7:13) < 0)
        psi2 = psi;
        O(added) = 0;
        [~, added] = max(O);
    else
        flag2 = 1;
    end
    end
    
    flag2 = 0;
    psi(n+1, :) = [configuration(added, :)];
    O = zeros(1, n_configuration);
   
    for i = 1:(n+1) % step c
        %i
        psi_deleted = psi;
        psi_deleted(i,:) = [];
        for j = 1:n
            % Observability Matrix
            J = Jacobian_parametric(psi_deleted(j,2:end), v, eps, option); 
            Jacobian_total(j*3-2:j*3,:) = J(1:3,:);  
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
        psi(deleted, :) = [];
        psi = sortrows(psi, 1);
        break
    end

    psi(deleted, :) = [];
    O = zeros(1, n_configuration);
    counter = counter + 1;
    toc
end

filename = sprintf('psi_loop.mat'); 
save(filename, "psi");
