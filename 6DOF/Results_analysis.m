%% Result analysis
clc
clear all

counter = 0;
load v_1
load v_2
load v_4
load goodconf_corrected.mat
goodconf = goodconf_corrected;
load measured_rotated.mat
v_0 = zeros(42,1);
%v_0(7:13) = -10^-8*ones(1,7);

for i = 51:height(goodconf)  
    counter = counter +1;
    theta = goodconf(i,2:7);
    P_t = FK(theta, v_0);
    P_t1 = FK(theta, v_1);
    P_t2 = FK(theta, v_2);
    P_t4 = FK(theta, v_4);
    P_m = -position_measured_rotated(i,:);
    P_m(end) = -P_m(end);
    error(counter) = norm(P_t(1:3) - P_m');
    error_corrected_1(counter) = norm(P_t1(1:3) - P_m');
    error_corrected_2(counter) = norm(P_t2(1:3) - P_m');
    error_corrected_4(counter) = norm(P_t4(1:3) - P_m');
end


% Generate errors histogram
figure(1)
hold on;
histogram(error, 10, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'r');
histogram(error_corrected_2, 10, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'b');
histogram(error_corrected_1, 10, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'g');
%histogram(error_corrected_4, 10, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'y');
hold off;
title('Histogram of the Position Errors');
xlabel('Error [mm]');
ylabel('Frequency');
legend('Nominal', 'Kinematic Calibration','Elasto-Kinematic Calibration');
grid on;

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
