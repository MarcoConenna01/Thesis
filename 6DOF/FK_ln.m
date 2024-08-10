function [P,t] = FK_ln(theta, v) 

    k = zeros(1,7);
    for i = 1:7
        k(i) = k(i) + v(i+6);
    end

    error = 100;
    [P, t] = End_effector_pos(theta, v);
    
    vv = v;
    while error > 10^-6
        P_0 = P;
        theta_new = theta + k(2:end).*t(2:end)';
        vv(36) = v(36) + k(1)*t(1);
        [P, t] = End_effector_pos(theta_new, vv);
        vv(36) = v(36);
        error = norm(abs(P-P_0));
    end
    
end

function [P, t] = End_effector_pos(theta, v)

    % geometric description of the arm
    d = [46.905 0 43.06953 0 43.25 0];
    dd = [103.505 107.75];
    a = [0 0 0 0 0 0];
    alpha = pi/2*[1 -1 1 -1 1 0];
    angle_gripper_alpha = -pi/2;
    angle_gripper_beta = 0;
    x_gripper = 0;
    y_gripper = 0;
    z_gripper = 130.38;
    angle_base_alpha = 0;
    angle_base_beta = pi/3;
    x_base = -34.975172;
    y_base = 0;
    z_base = 236.08235;

    % calculation for the cm of target + weight
    cm_target = [24.32 2.1 4.33];
    cm_w = [0 0 15];
    m_target = 48.2;
    m_w = 9.2; % screw
    %m_w = 9.2 + 120.8; %heaviest weight
    m_tot = m_w + m_target;
    cm_tot = (cm_target*m_target + cm_w*m_w)/m_tot;

    cm = [ 0 2 37.32; ... %1 DOF
           12.46484 -0.20331 47.6142; ...
           0 2.5 31.9581; ...
           4.5466 0 53.125; ...
           0 0 29.164; ...
           0 0 45.12;
           cm_tot]; 
    m = [123.4 177 101.6 132.8 106 81 m_tot];

    % add the calibration parameters to the geometric parameters
    alpha(1) = alpha(1) + v(1);
    alpha(2) = alpha(2) + v(2);
    alpha(3) = alpha(3) + v(3);
    alpha(4) = alpha(4) + v(4);
    alpha(5) = alpha(5) + v(5);
    alpha(6) = alpha(6) + v(6);
    a(1) = a(1) + v(14);
    a(2) = a(2) + v(15);
    a(3) = a(3) + v(16);
    a(4) = a(4) + v(17);
    a(5) = a(5) + v(18);
    a(6) = a(6) + v(19);
    theta(1) = theta(1) + v(20);
    theta(2) = theta(2) + v(21);
    theta(3) = theta(3) + v(22);
    theta(4) = theta(4) + v(23);
    theta(5) = theta(5) + v(24);
    theta(6) = theta(6) + v(25);
    d(1) = d(1) + v(26);
    d(2) = d(2) + v(27);
    d(3) = d(3) + v(28);
    d(4) = d(4) + v(29);
    d(5) = d(5) + v(30);
    d(6) = d(6) + v(31);
    x_base = x_base + v(32);
    y_base = y_base + v(33);
    z_base = z_base + v(34);
    angle_base_alpha = angle_base_alpha + v(35);
    angle_base_beta = angle_base_beta + v(36);
    x_gripper = x_gripper + v(37);
    y_gripper = y_gripper + v(38);
    z_gripper = z_gripper + v(39);
    angle_gripper_alpha = angle_gripper_alpha + v(40);
    angle_gripper_beta = angle_gripper_beta + v(41);
    dd(1) = dd(1) + v(42);

    % base translation and rotation
    A_transl = [1 0 0 x_base; 0 1 0 y_base; 0 0 1 z_base; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_base_alpha) -sin(angle_base_alpha) 0; 0 sin(angle_base_alpha) cos(angle_base_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_base_beta) 0 sin(angle_base_beta) 0; 0 1 0 0; -sin(angle_base_beta) 0 cos(angle_base_beta) 0; 0 0 0 1];
    A_base = A_transl*Aalpha*Abeta;

    % 3rd and 5th joint extra y translations
    trans_3 = [1 0 0 0; 0 1 0 0; 0 0 1 dd(1); 0 0 0 1];
    trans_5 = [1 0 0 0; 0 1 0 0; 0 0 1 dd(2); 0 0 0 1];

    % end-effector translation and rotation
    A_transl = [1 0 0 x_gripper; 0 1 0 y_gripper; 0 0 1 z_gripper; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_gripper_alpha) -sin(angle_gripper_alpha) 0; 0 sin(angle_gripper_alpha) cos(angle_gripper_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_gripper_beta) 0 sin(angle_gripper_beta) 0; 0 1 0 0; -sin(angle_gripper_beta) 0 cos(angle_gripper_beta) 0; 0 0 0 1];
    A = Aalpha*Abeta*A_transl;
    
    % Compute transformation matrices
    A01 = buildHD(theta(1)+pi, alpha(1), d(1), a(1));
    A12 = buildHD(theta(2), alpha(2), d(2), a(2));
    A23 = buildHD(theta(3), alpha(3), d(3), a(3));
    A34 = buildHD(theta(4), alpha(4), d(4), a(4));
    A45 = buildHD(theta(5), alpha(5), d(5), a(5));
    A56 = buildHD(theta(6), alpha(6), d(6), a(6));
    
    % Compute positions of each joint
    T01 = A_base;
    T02 = A_base * A01;
    T03 = T02 * A12 * trans_3;
    T04 = T03 * A23;
    T05 = T04 * A34 * trans_5 ;
    T06 = T05 * A45;
    Tgripper = T06 * A56;
    Tgripper = Tgripper * A;
    P = Tgripper(1:3, 4);

    % Extract joint positions
    pos1 = A_base(1:3, 4);
    pos2 = T02(1:3, 4);
    pos3 = T03(1:3, 4);
    pos4 = T04(1:3, 4);
    pos5 = T05(1:3, 4);
    pos6 = T06(1:3, 4);
    posgripper = Tgripper(1:3, 4);

    P(4:6) = rotmat2vec3d(Tgripper(1:3,1:3));   % test: adding orientation gripper
    %P(7:24) = [pos1; pos2; pos3; pos4; pos5; pos6];
 
    % Compute positions of centers of mass of each joint
    cm01 = A_base * [1 0 0 cm(1,1); 0 1 0 cm(1,2); 0 0 1 cm(1,3); 0 0 0 1];
    cm02 = T03 * [1 0 0 cm(2,1); 0 1 0 cm(2,2); 0 0 1 cm(2,3) - dd(1); 0 0 0 1];    
    cm03 = T03 * [1 0 0 cm(3,1); 0 1 0 cm(3,2); 0 0 1 cm(3,3); 0 0 0 1];    
    cm04 = T05 * [1 0 0 cm(4,1); 0 1 0 cm(4,2); 0 0 1 cm(4,3) - dd(2); 0 0 0 1];   
    cm05 = T05 * [1 0 0 cm(5,1); 0 1 0 cm(5,2); 0 0 1 cm(5,3); 0 0 0 1];   
    cm06 = Tgripper * [1 0 0 cm(6,1); 0 1 0 cm(6,2); 0 0 1 cm(6,3) - z_gripper; 0 0 0 1];    
    cm07 = Tgripper * [1 0 0 cm(7,1); 0 1 0 cm(7,2); 0 0 1 cm(7,3); 0 0 0 1];  

    % Extract centers of mass positions
    poscm01 = cm01(1:3, 4);
    poscm02 = cm02(1:3, 4);
    poscm03 = cm03(1:3, 4);
    poscm04 = cm04(1:3, 4);
    poscm05 = cm05(1:3, 4);
    poscm06 = cm06(1:3, 4);
    poscm07 = cm07(1:3, 4);

    % Calculate torque acting on each joint
    t6 =  cross((poscm07 - pos6), [0 0 -9.81*m(7)]) + cross((poscm06 - pos6), [0 0 -9.81*m(6)]);
    rot = T06(1:3,1:3) * [0 0 1]';
    t6_theta = t6*rot;

    t5 = cross((poscm07 - pos5), [0 0 -9.81*m(7)]) + cross((poscm06 - pos5), [0 0 -9.81*m(6)]) + cross((poscm05 - pos5), [0 0 -9.81*m(5)]);
    rot = T05(1:3,1:3) * [0 0 1]';
    t5_theta = t5*rot;

    t4 = cross((poscm07 - pos4), [0 0 -9.81*m(7)]) + cross((poscm06 - pos4), [0 0 -9.81*m(6)]) + cross((poscm05 - pos4), [0 0 -9.81*m(5)]) + cross((poscm04 - pos4), [0 0 -9.81*m(4)]);
    rot = T04(1:3,1:3) * [0 0 1]';
    t4_theta = t4*rot;
 
    t3 = cross((poscm07 - pos3), [0 0 -9.81*m(7)]) + cross((poscm06 - pos3), [0 0 -9.81*m(6)]) + cross((poscm05 - pos3), [0 0 -9.81*m(5)]) + cross((poscm04 - pos3), [0 0 -9.81*m(4)]) + cross((poscm03 - pos3), [0 0 -9.81*m(3)]);
    rot = T03(1:3,1:3) * [0 0 1]';
    t3_theta = t3*rot;

    t2 = cross((poscm07 - pos2), [0 0 -9.81*m(7)]) + cross((poscm06 - pos2), [0 0 -9.81*m(6)]) + cross((poscm05 - pos2), [0 0 -9.81*m(5)]) + cross((poscm04 - pos2), [0 0 -9.81*m(4)]) + cross((poscm03 - pos2), [0 0 -9.81*m(3)]) + cross((poscm02 - pos2), [0 0 -9.81*m(2)]);
    rot = T02(1:3,1:3) * [0 0 1]';
    t2_theta = t2*rot;

    t1 = cross((poscm07 - pos1), [0 0 -9.81*m(7)]) + cross((poscm06 - pos1), [0 0 -9.81*m(6)]) + cross((poscm05 - pos1), [0 0 -9.81*m(5)]) + cross((poscm04 - pos1), [0 0 -9.81*m(4)]) + cross((poscm03 - pos1), [0 0 -9.81*m(3)]) + cross((poscm02 - pos1), [0 0 -9.81*m(2)]) + cross((poscm01 - pos1), [0 0 -9.81*m(1)]);
    rot = T01(1:3,1:3) * [0 0 1]';
    t1_theta = t1*rot;

    rot = A_base(1:3,1:3) * [0 1 0]';
    t0_beta = t1*rot;
    
    t = [t0_beta; t1_theta; t2_theta; t3_theta; t4_theta; t5_theta; t6_theta];
    
    % Vector of joints position
    posx = [0 pos1(1) pos2(1) pos3(1) pos4(1) pos5(1) pos6(1) posgripper(1)];
    posy = [0 pos1(2) pos2(2) pos3(2) pos4(2) pos5(2) pos6(2) posgripper(2)];
    posz = [0 pos1(3) pos2(3) pos3(3) pos4(3) pos5(3) pos6(3) posgripper(3)];

    % Vector of cm positions
    posxx = [poscm01(1) poscm02(1) poscm03(1) poscm04(1) poscm05(1) poscm06(1) poscm07(1)];
    posyy = [poscm01(2) poscm02(2) poscm03(2) poscm04(2) poscm05(2) poscm06(2) poscm07(2)];
    poszz = [poscm01(3) poscm02(3) poscm03(3) poscm04(3) poscm05(3) poscm06(3) poscm07(3)];
    
%     % Plot the points as red big dots with a line connecting them
%     figure();
%     scatter3(posx, posy, posz, 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
%     text(posx, posy, posz, ['0';'1';'2';'3';'4';'5';'6';'7'])
%     grid on;
%     axis equal
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
%     title('3D Coordinates');
%     hold on;
%     
%     % Plot the centers of mass
%     scatter3(posxx, posyy, poszz, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
%     text(posxx, posyy, poszz, ['cm01';'cm02';'cm03';'cm04';'cm05';'cm06';'cm07'])
%     
%     % Plot reference frames at each joint
%     scale = 20; % Scale for the quiver arrows
%     plotFrame(A_base, scale);
%     plotFrame(T02, scale);
%     plotFrame(T03, scale);
%     plotFrame(T04, scale);
%     plotFrame(T05, scale);
%     plotFrame(T06, scale);
%     plotFrame(Tgripper, scale);

end

function plotFrame(T, scale)
    origin = T(1:3, 4)';
    x_axis = T(1:3, 1)' * scale;
    y_axis = T(1:3, 2)' * scale;
    z_axis = T(1:3, 3)' * scale;
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 'r');
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 'g');
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 'b');
end

% Define the buildHD function
function [A] = buildHD(theta, alpha, d, a)
    A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);...
       sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
       0 sin(alpha) cos(alpha) d;...
       0 0 0 1];
end
