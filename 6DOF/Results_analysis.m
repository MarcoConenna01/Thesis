%% Result analysis
clc
clear all

counter = 0;
load v_1
load v_2
load v_3
load v_4
load v_loop
load LRM_ARM_data/goodconf_corrected.mat
goodconf = goodconf_corrected;
load LRM_ARM_data/measured_rotated.mat
v_0 = zeros(42,1);


load psi_1
load psi_2
load psi_3
load psi_4
load psi_loop

anti_psi_1 = goodconf;
anti_psi_2 = goodconf;
anti_psi_3 = goodconf;
anti_psi_4 = goodconf;
anti_psi_loop = goodconf;
for i = 1:height(goodconf)
    if ismember(goodconf(i,1), psi_1(:,1))
        anti_psi_1(i,:) = zeros(7,1);
    end
    if ismember(goodconf(i,1), psi_2(:,1))
        anti_psi_2(i,:) = zeros(7,1);
    end
    if ismember(goodconf(i,1), psi_3(:,1))
        anti_psi_3(i,:) = zeros(7,1);
    end
    if ismember(goodconf(i,1), psi_4(:,1))
        anti_psi_4(i,:) = zeros(7,1);
    end
    if ismember(goodconf(i,1), psi_loop(:,1))
        anti_psi_loop(i,:) = zeros(7,1);
    end     
end

anti_psi_1(all(anti_psi_1 == 0, 2), :) = [];
anti_psi_2(all(anti_psi_2 == 0, 2), :) = [];
anti_psi_3(all(anti_psi_3 == 0, 2), :) = [];
anti_psi_4(all(anti_psi_4 == 0, 2), :) = [];
anti_psi_loop(all(anti_psi_loop == 0, 2), :) = [];

for i = 1:height(goodconf)  
    counter = counter +1;
    theta = goodconf(i,2:7);
    P_t = FK(theta, v_0);
    P_t1 = FK(theta, v_1);
    P_t2 = FK(theta, v_2);
    P_t3 = FK(theta, v_3);
    P_t4 = FK(theta, v_4);
    P_tloop = FK(theta, v_loop);
    P_m = -position_measured_rotated(i,:);
    P_m(end) = -P_m(end);
    error(counter) = norm(P_t(1:3) - P_m');
    error_corrected_1(counter) = norm(P_t1(1:3) - P_m');
    error_corrected_2(counter) = norm(P_t2(1:3) - P_m');
    error_corrected_3(counter) = norm(P_t3(1:3) - P_m');
    error_corrected_4(counter) = norm(P_t4(1:3) - P_m');
    error_corrected_loop(counter) = norm(P_tloop(1:3) - P_m');
end

counter = 0;
for i = 1:height(goodconf) - 50
    counter = counter +1;

    theta1 = anti_psi_1(i,2:end);
    theta2 = anti_psi_2(i,2:end);
    theta3 = anti_psi_3(i,2:end);
    theta4 = anti_psi_4(i,2:end);
    thetaloop = anti_psi_loop(i,2:end);
    P_not_1 = FK(theta1, v_0);
    P_not_2 = FK(theta2, v_0);
    P_not_3 = FK(theta3, v_0);
    P_not_4 = FK(theta4, v_0);
    P_not_loop = FK(thetaloop, v_0);
    P_m1 = -position_measured_rotated(anti_psi_1(i,1),:);
    P_m1(end) = -P_m1(end);
    P_m2 = -position_measured_rotated(anti_psi_2(i,1),:);
    P_m2(end) = -P_m2(end);
    P_m3 = -position_measured_rotated(anti_psi_3(i,1),:);
    P_m3(end) = -P_m3(end);
    P_m4 = -position_measured_rotated(anti_psi_4(i,1),:);
    P_m4(end) = -P_m4(end);
    P_mloop = -position_measured_rotated(anti_psi_loop(i,1),:);
    P_mloop(end) = -P_mloop(end);   

    P_t1 = FK(theta1, v_1);
    P_t2 = FK(theta2, v_2);
    P_t3 = FK(theta3, v_3);
    P_t4 = FK(theta4, v_4);
    P_tloop = FK(thetaloop, v_loop);

    error_corrected_1_not(counter) = norm(P_t1(1:3) - P_m1');
    error_corrected_2_not(counter) = norm(P_t2(1:3) - P_m2');
    error_corrected_3_not(counter) = norm(P_t3(1:3) - P_m3');
    error_corrected_4_not(counter) = norm(P_t4(1:3) - P_m4');
    error_corrected_loop_not(counter) = norm(P_tloop(1:3) - P_mloop');
end


% % Generate errors histogram
% figure(1)
% hold on;
% histogram(error, 10, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'r');
% histogram(error_corrected_2, 10, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'b');
% histogram(error_corrected_1, 10, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'g');
% %histogram(error_corrected_4, 10, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'y');
% hold off;
% title('Histogram of the Position Errors');
% xlabel('Error [mm]');
% ylabel('Frequency');
% legend('Nominal', 'Kinematic Calibration','Elasto-Kinematic Calibration');
% grid on;

% % Generate errors distributions plot
% figure(2)
% hold on
% x = linspace(0, 7, 1000);
% Gaussian_nominal = gaussian(x, mean(error), std(error));
% Gaussian_kinematic = gaussian(x, mean(error4), std(error4));
% Gaussian_elasto_kinematic = gaussian(x, mean(error2), std(error2));
% Gaussian_elastic = gaussian(x, mean(error5), std(error5));
% plot(x, Gaussian_nominal,'r', 'LineWidth',1.5)
% plot(x, Gaussian_kinematic,'Color',[0 0.8 0], 'LineWidth',1.5)
% plot(x, Gaussian_elasto_kinematic,'b', 'LineWidth',1.5)
% plot(x, Gaussian_elastic,'Color', [1 0.8 0], 'LineWidth',1.5)
% hold off;
% title('Gaussian Distrubutions of the Position Errors');
% xlabel('Error [mm]');
% ylabel('Probability');
% legend('Nominal', 'Kinematic Calibration','Elasto-Kinematic Calibration', 'Elastic Calibration');
% grid on;
% 
% function y = gaussian(x, mu, sigma)
% 
% a = (-(x-mu).^2./(2*sigma^2));
% y = 1/(sigma*sqrt(2*pi))*exp(a);
% 
% end
% 
