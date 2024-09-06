%% Result analysis
clc
clear all

counter = 0;
load LRM_ARM_data/v_1
load LRM_ARM_data/v_2
load LRM_ARM_data/v_3
load LRM_ARM_data/v_4
load LRM_ARM_data/v_5
load LRM_ARM_data/v_loop
load LRM_ARM_data/v_loop_5
load LRM_ARM_data/goodconf_corrected.mat
goodconf = goodconf_corrected;
load LRM_ARM_data/measured_rotated.mat
v_0 = zeros(42,1);


load LRM_ARM_data/psi_1
load LRM_ARM_data/psi_2
load LRM_ARM_data/psi_3
load LRM_ARM_data/psi_4
load LRM_ARM_data/psi_5
load LRM_ARM_data/psi_loop
load LRM_ARM_data/psi_loop_5

anti_psi_1 = goodconf;
anti_psi_2 = goodconf;
anti_psi_3 = goodconf;
anti_psi_4 = goodconf;
anti_psi_5 = goodconf;
anti_psi_loop = goodconf;
anti_psi_loop5 = goodconf;
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
    if ismember(goodconf(i,1), psi_5(:,1))
        anti_psi_5(i,:) = zeros(7,1);
    end
    if ismember(goodconf(i,1), psi_loop(:,1))
        anti_psi_loop(i,:) = zeros(7,1);
    end     
    if ismember(goodconf(i,1), psi_loop_5(:,1))
        anti_psi_loop5(i,:) = zeros(7,1);
    end     
end

anti_psi_1(all(anti_psi_1 == 0, 2), :) = [];
anti_psi_2(all(anti_psi_2 == 0, 2), :) = [];
anti_psi_3(all(anti_psi_3 == 0, 2), :) = [];
anti_psi_4(all(anti_psi_4 == 0, 2), :) = [];
anti_psi_5(all(anti_psi_5 == 0, 2), :) = [];
anti_psi_loop(all(anti_psi_loop == 0, 2), :) = [];
anti_psi_loop5(all(anti_psi_loop5 == 0, 2), :) = [];

for i = 1:height(goodconf)  
    counter = counter +1;
    theta = goodconf(i,2:7);
    P_t = FK(theta, v_0);
    P_t1 = FK(theta, v_1);
    P_t2 = FK(theta, v_2);
    P_t3 = FK(theta, v_3);
    P_t4 = FK(theta, v_4);
    P_t5 = FK(theta, v_5);
    P_tloop = FK(theta, v_loop);
    P_tloop5 = FK(theta, v_loop_5);
    P_m = -position_measured_rotated(i,:);
    P_m(end) = -P_m(end);
    error(counter) = norm(P_t(1:3) - P_m');
    error_corrected_1(counter) = norm(P_t1(1:3) - P_m');
    error_corrected_2(counter) = norm(P_t2(1:3) - P_m');
    error_corrected_3(counter) = norm(P_t3(1:3) - P_m');
    error_corrected_4(counter) = norm(P_t4(1:3) - P_m');
    error_corrected_5(counter) = norm(P_t5(1:3) - P_m');
    error_corrected_loop(counter) = norm(P_tloop(1:3) - P_m');
    error_corrected_loop_5(counter) = norm(P_tloop5(1:3) - P_m');
end

counter = 0;
for i = 1:height(goodconf) - 50
    counter = counter +1;

    theta1 = anti_psi_1(i,2:end);
    theta2 = anti_psi_2(i,2:end);
    theta3 = anti_psi_3(i,2:end);
    theta4 = anti_psi_4(i,2:end);
    theta5 = anti_psi_5(i,2:end);
    thetaloop = anti_psi_loop(i,2:end);
    thetaloop5 = anti_psi_loop5(i,2:end);
    P_not_1 = FK(theta1, v_0);
    P_not_2 = FK(theta2, v_0);
    P_not_3 = FK(theta3, v_0);
    P_not_4 = FK(theta4, v_0);
    P_not_5 = FK(theta5, v_0);
    P_not_loop = FK(thetaloop, v_0);
    P_not_loop5 = FK(thetaloop5, v_0);
    P_m1 = -position_measured_rotated(anti_psi_1(i,1),:);
    P_m1(end) = -P_m1(end);
    P_m2 = -position_measured_rotated(anti_psi_2(i,1),:);
    P_m2(end) = -P_m2(end);
    P_m3 = -position_measured_rotated(anti_psi_3(i,1),:);
    P_m3(end) = -P_m3(end);
    P_m4 = -position_measured_rotated(anti_psi_4(i,1),:);
    P_m4(end) = -P_m4(end);
    P_m5 = -position_measured_rotated(anti_psi_5(i,1),:);
    P_m5(end) = -P_m5(end);
    P_mloop = -position_measured_rotated(anti_psi_loop(i,1),:);
    P_mloop(end) = -P_mloop(end);   
    P_mloop5 = -position_measured_rotated(anti_psi_loop5(i,1),:);
    P_mloop5(end) = -P_mloop5(end); 

    P_t1 = FK(theta1, v_1);
    P_t2 = FK(theta2, v_2);
    P_t3 = FK(theta3, v_3);
    P_t4 = FK(theta4, v_4);
    P_t5 = FK(theta5, v_5);
    P_tloop = FK(thetaloop, v_loop);
    P_tloop5 = FK(thetaloop5, v_loop_5);

    error_corrected_1_not(counter) = norm(P_t1(1:3) - P_m1');
    error_corrected_2_not(counter) = norm(P_t2(1:3) - P_m2');
    error_corrected_3_not(counter) = norm(P_t3(1:3) - P_m3');
    error_corrected_4_not(counter) = norm(P_t4(1:3) - P_m4');
    error_corrected_5_not(counter) = norm(P_t5(1:3) - P_m5');
    error_corrected_loop_not(counter) = norm(P_tloop(1:3) - P_mloop');
    error_corrected_loop_not5(counter) = norm(P_tloop5(1:3) - P_mloop5');
end

%% PLOTS

% Gaussian with all the options
figure(1)
hold on
x = linspace(0, 40, 1000);
Gaussian_nominal = gaussian(x, mean(error), std(error));
Gaussian_1 = gaussian(x, mean(error_corrected_1), std(error_corrected_1));
Gaussian_2 = gaussian(x, mean(error_corrected_2), std(error_corrected_2));
Gaussian_3 = gaussian(x, mean(error_corrected_3), std(error_corrected_3));
Gaussian_4 = gaussian(x, mean(error_corrected_4), std(error_corrected_4));
Gaussian_5 = gaussian(x, mean(error_corrected_5), std(error_corrected_5));
Gaussian_1l = gaussian(x, mean(error_corrected_loop), std(error_corrected_loop));
Gaussian_5l = gaussian(x, mean(error_corrected_loop_5), std(error_corrected_loop_5));
plot(x, Gaussian_nominal, 'k', 'LineWidth',3)
plot(x, Gaussian_1,'Color',[0.4941, 0.1765, 0.5569], 'LineWidth',2.5)
plot(x, Gaussian_2,'Color',[0.8549, 0.3255, 0.0980] , 'LineWidth',2.5)
plot(x, Gaussian_3,'Color', [0.4667, 0.6745, 0.1882],'LineWidth',2)
plot(x, Gaussian_4,'Color',[0.3020, 0.7410, 0.9333] ,'LineWidth',2)
plot(x, Gaussian_5,'Color', [0, 0.4470, 0.7410],'LineWidth',2)
plot(x, Gaussian_1l,'Color', [0.4941, 0.1765, 0.5569],'LineStyle','--' , 'LineWidth',1.5)
plot(x, Gaussian_5l,'Color', [0, 0.4470, 0.7410], 'LineStyle','--' ,'LineWidth',1.5)
hold off;
title('Gaussian Distrubutions of the TCP Position Errors for each Parametrization Option');
xlabel('Error = ||x_m - x_{est}|| [mm]');
ylabel('Probability');
legend('Nominal', 'Elasto-Kinematic Calibration (Option 1)','Kinematic Calibration  (Option 2)', 'Elastic Calibration  (Option 3)','Option 4',' Option 5', 'Option 1_2', 'Option 5_2');
grid on;

% Gaussian with all the options, for the 343 poses
figure(2)
hold on
x = linspace(0, 40, 1000);
Gaussian_nominal = gaussian(x, mean(error), std(error));
Gaussian_1 = gaussian(x, mean(error_corrected_1_not), std(error_corrected_1_not));
Gaussian_2 = gaussian(x, mean(error_corrected_2_not), std(error_corrected_2_not));
Gaussian_3 = gaussian(x, mean(error_corrected_3_not), std(error_corrected_3_not));
Gaussian_4 = gaussian(x, mean(error_corrected_4_not), std(error_corrected_4_not));
Gaussian_5 = gaussian(x, mean(error_corrected_5_not), std(error_corrected_5_not));
Gaussian_1l = gaussian(x, mean(error_corrected_loop_not), std(error_corrected_loop));
Gaussian_5l = gaussian(x, mean(error_corrected_loop_not5), std(error_corrected_loop_not5));
plot(x, Gaussian_nominal, 'k', 'LineWidth',3)
plot(x, Gaussian_1,'Color',[0.4941, 0.1765, 0.5569], 'LineWidth',2.5)
plot(x, Gaussian_2,'Color',[0.8549, 0.3255, 0.0980] , 'LineWidth',2.5)
plot(x, Gaussian_3,'Color', [0.4667, 0.6745, 0.1882],'LineWidth',2)
plot(x, Gaussian_4,'Color',[0.3020, 0.7410, 0.9333] ,'LineWidth',2)
plot(x, Gaussian_5,'Color', [0, 0.4470, 0.7410],'LineWidth',2)
plot(x, Gaussian_1l,'Color', [0.4941, 0.1765, 0.5569],'LineStyle','--' , 'LineWidth',1.5)
plot(x, Gaussian_5l,'Color', [0, 0.4470, 0.7410], 'LineStyle','--' ,'LineWidth',1.5)
hold off;
title('Gaussian Distrubutions of the TCP Position Errors for each Parametrization Option | not in \Psi');
xlabel('Error = ||x_m - x_{est}|| [mm]');
ylabel('Probability');
legend('Nominal', 'Elasto-Kinematic Calibration (Option 1)','Kinematic Calibration  (Option 2)', 'Elastic Calibration  (Option 3)','Option 4',' Option 5', 'Option 1_2', 'Option 5_2');
grid on;


% Generate errors histogram
figure(3)
hold on;
histogram(error, 10, 'FaceColor', 'k', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
histogram(error_corrected_loop_not, 10, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'r');
hold off;
title('Histogram of the TCP Position Errors');
xlabel('Error [mm]');
ylabel('Frequency');
legend('Nominal', 'Elasto-Kinematic Calibration (Option 1_2) not in \Psi');
grid on;




function y = gaussian(x, mu, sigma)

a = (-(x-mu).^2./(2*sigma^2));
y = 1/(sigma*sqrt(2*pi))*exp(a);

end

