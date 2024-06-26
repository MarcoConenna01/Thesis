clc
clear all
close all

%% List of possible conf-wrench combinations

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
n = 20;
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
                JJ = Jacobian_parametric_2DOF(psi(j,2:3));
                %Jacobian_total(j*3-2:j*3,:) = JJ(1:3,:);
                %adding orientation of the end effector
                Jacobian_total(j*6-5:j*6,:) = JJ;
            end
            % calculate index
            S = svd(Jacobian_total);
            %O(i) = (prod(S))^(1/length(S))/(sqrt(n+1));
            O(i) = S(end)^2/S(1);
            clear Jacobian_total
        end
    end

    % check which gave the best O (observability index) and add it to the conf, then do the same
    % thing deleting one
    [~, added] = max(O)
    psi(n+1, :) = [configuration(added, :)];
    O = zeros(1, n_configuration);
   
    for i = 1:(n+1) % step c
        %i
        psi_deleted = psi;
        psi_deleted(i,:) = [];
        for j = 1:n
            % Observability Matrix
            JJ = Jacobian_parametric_2DOF(psi_deleted(j,2:3)); 
            %Jacobian_total(j*3-2:j*3,:) = JJ(1:3,:);
            %adding orientation of the end effector
            Jacobian_total(j*6-5:j*6,:) = JJ; 
        end

            % calculate index
            S = svd(Jacobian_total);
            %O(i) = (prod(S))^(1/length(S))/(sqrt(n));
            O(i) = S(end)^2/S(1);
            clear Jacobian_total
       
    end
    
    [~, deleted] = max(O)

     % exit condition, deleted == added
    if psi(deleted,1) == added
        flag = 1;
    end

    psi(deleted, :) = [];
    O = zeros(1, n_configuration);
    counter = counter + 1;
end




