clc
clear all
close all

% List of possible conf-wrench combinations
n_parameters = 42;
v = zeros(1,n_parameters);
eps = 10^-9;

load LRM_ARM_data/goodconf_corrected.mat
configuration = goodconf_corrected;
n_configuration = height(configuration);

for big_loop = 5 
   option = big_loop;

    % Observability Matrix
    counter = 0;
    flag = 0;
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
                    %adding orientation of the end effector
                    %Jacobian_total(j*6-5:j*6,:) = JJ;
                end
                % calculate index
                S = svd(Jacobian_total);
                O(i) = (prod(S))^(1/length(S))/(sqrt(n+1));
                clear Jacobian_total
            end
        end
    
        % check which gave the best O (observability index) and add it to the conf, then do the same
        % thing deleting one
        [~, added] = max(O);
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
                %adding orientation of the end effector
                %Jacobian_total(j*6-5:j*6,:) = JJ; 
            end
            % calculate index
            S = svd(Jacobian_total);
            O(i) = (prod(S))^(1/length(S))/(sqrt(n));
            JJJ = Jacobian_total;
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

filename = sprintf('psi_%d.mat', big_loop);  % Creates filenames like file_1.mat, file_2.mat, etc.
save(filename, "psi");
end