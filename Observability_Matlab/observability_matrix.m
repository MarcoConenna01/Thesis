clc
clear all
close all

%% List of possible conf-wrench combinations (TODO: ADD WRENCH)

n_theta = 4;
n_configuration = n_theta^6;
field = linspace(-10*pi/18,10*pi/18,n_theta);
counter = 1;
configuration = zeros(n_configuration,7);

for i = 1:n_theta
    for j = 1:n_theta
        for k = 1:n_theta
            for l = 1:n_theta
                for z = 1:n_theta
                    for x = 1:n_theta

                        configuration(counter,:) = [counter, field(i), field(j), field(k), field(l), field(z), field(x)];
                        counter = counter +1

                    end
                end
            end
        end
    end
end


flag = 0;
n = 50;
psi = configuration(1:n,:);
O = zeros(1, n_configuration);

while flag == 0

    for i = 1:(n_configuration) % step b
        i
        if ismember(configuration(i, 1), psi(:,1)) 

        else

        psi(n+1,:) = configuration(i, :);

        for j = 1:(n+1)
            
            % Observability Matrix
            Jacobian_total(j*3-2:j*3,:) = Jacobian_parametric(psi(j,2:7));

        end

        % calculate index
        S = svd(Jacobian_total);
        O(i) = (prod(S))^(1/length(S))/(sqrt(n+1));
        end
    end

    % check which gave the best O and add it to the conf, then do the same
    % thing deleting one

    flag = 1;
end


