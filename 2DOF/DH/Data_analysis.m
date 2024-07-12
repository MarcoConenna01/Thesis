clc
clearvars
close all

%% Measurements
load lrm_arm_marker_03_07
position_arm = [pose_position_x' pose_position_y' pose_position_z'];

load lrm_base_marker_03_07.mat
position_base = [pose_position_x' pose_position_y' pose_position_z'];

position = 1000*(position_arm - position_base);
position_measured = zeros(100,3);
orientation_measured = zeros(100,4);
position_measured_rotated = zeros(100,3);
for i = 1:100
    upperlimit = 300 + 750*(i-1);
    lowerlimit = upperlimit - 100;
    position_measured(i,:) = [mean(position(lowerlimit:upperlimit,1)),mean(position(lowerlimit:upperlimit,2)),mean(position(lowerlimit:upperlimit,3))];
    orientation_measured(i,:) = [mean(pose_orientation_w(lowerlimit:upperlimit)),mean(pose_orientation_x(lowerlimit:upperlimit)),mean(pose_orientation_y(lowerlimit:upperlimit)),mean(pose_orientation_z(lowerlimit:upperlimit))];
    rotation_matrix = quat2rotm(orientation_measured(i,:));
    position_measured_rotated(i,:) = rotation_matrix' * position_measured(i,:)';
end

%% Result analysis
load v_2
load v_4
load v_5
n_theta = 10;
n_configuration = n_theta^2;
field = linspace(-10*pi/18,10*pi/18,n_theta);
counter = 0;
configuration = zeros(n_configuration,3);
error= zeros(1,height(configuration));
error2 = zeros(1,height(configuration));
error3 = zeros(1,height(configuration));
error4 = zeros(1,height(configuration));
error5 = zeros(1,height(configuration));
v_0 = zeros(27,1);
v_0(7:9) = -10^-8;
v_4(7:9) = -10^-8;
for i = 1:n_theta
    for j = 1:n_theta    
        counter = counter +1;
        P_t = FK_2DOF([field(i) field(j)], v_0);
        P_t2 = FK_2DOF([field(i) field(j)], v);
        P_t5 = FK_2DOF([field(i) field(j)], v_5);
        P_t4 = FK_2DOF([field(i) field(j)], v_4);
        P_m = -position_measured_rotated(counter,:);
        P_m(end) = -P_m(end);
        error(counter) = norm(P_t(1:3) - P_m');
        error2(counter) = norm(P_t2(1:3) - P_m');
        error5(counter) = norm(P_t5(1:3) - P_m');
        error4(counter) = norm(P_t4(1:3) - P_m');
    end
end

% Generate errors histogram
figure(1)
hold on;
histogram(error, 10, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'r');
histogram(error4, 10, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'g');
histogram(error2, 10, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'b');
%histogram(error5, 10, 'FaceColor', 'y', 'FaceAlpha', 0.5, 'EdgeColor', 'y');
hold off;
title('Histogram of the Position Errors');
xlabel('Error [mm]');
ylabel('Frequency');
legend('Nominal', 'Kinematic Calibration','Elasto-Kinematic Calibration');
grid on;

% Generate errors distributions plot
figure(2)
hold on
x = linspace(0, 7, 1000);
Gaussian_nominal = gaussian(x, mean(error), std(error));
Gaussian_kinematic = gaussian(x, mean(error4), std(error4));
Gaussian_elasto_kinematic = gaussian(x, mean(error2), std(error2));
Gaussian_elastic = gaussian(x, mean(error5), std(error5));
plot(x, Gaussian_nominal,'r', 'LineWidth',1.5)
plot(x, Gaussian_kinematic,'Color',[0 0.8 0], 'LineWidth',1.5)
plot(x, Gaussian_elasto_kinematic,'b', 'LineWidth',1.5)
plot(x, Gaussian_elastic,'Color', [1 0.8 0], 'LineWidth',1.5)
hold off;
title('Gaussian Distrubutions of the Position Errors');
xlabel('Error [mm]');
ylabel('Probability');
legend('Nominal', 'Kinematic Calibration','Elasto-Kinematic Calibration', 'Elastic Calibration');
grid on;

function y = gaussian(x, mu, sigma)

a = (-(x-mu).^2./(2*sigma^2));
y = 1/(sigma*sqrt(2*pi))*exp(a);

end

