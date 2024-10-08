function [P,t] = End_effector_pos_2DOF(m, cm, theta, d, a, alpha, x_gripper, y_gripper, z_gripper, x_base, y_base, z_base, angle_gripper_alpha, angle_gripper_beta, angle_base_alpha, angle_base_beta)

    % end-effector translation and rotation
    A_transl = [1 0 0 x_gripper; 0 1 0 y_gripper; 0 0 1 z_gripper; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_gripper_alpha) -sin(angle_gripper_alpha) 0; 0 sin(angle_gripper_alpha) cos(angle_gripper_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_gripper_beta) 0 sin(angle_gripper_beta) 0; 0 1 0 0; -sin(angle_gripper_beta) 0 cos(angle_gripper_beta) 0; 0 0 0 1];
    A = Aalpha*Abeta*A_transl;
    
    % base translation and rotation
    A_transl = [1 0 0 x_base; 0 1 0 y_base; 0 0 1 z_base; 0 0 0 1];
    Aalpha = [1 0 0 0; 0 cos(angle_base_alpha) -sin(angle_base_alpha) 0; 0 sin(angle_base_alpha) cos(angle_base_alpha) 0; 0 0 0 1];
    Abeta = [cos(angle_base_beta) 0 sin(angle_base_beta) 0; 0 1 0 0; -sin(angle_base_beta) 0 cos(angle_base_beta) 0; 0 0 0 1];
    A_base = A_transl*Abeta*Aalpha;
    
    % Compute transformation matrices
    A01 = buildHD(theta(1), alpha(1), d(1), a(1));
    A12 = buildHD(theta(2), alpha(2), d(2), a(2));
    
    % Compute positions of each joint
    T01 = A_base * A01;
    T02 = T01 * A12;
    Tgripper = T02 * A;
    P = Tgripper(1:3, 4);

    % Extract joint positions
    pos0 = A_base(1:3, 4);
    pos1 = T01(1:3, 4);
    pos2 = T02(1:3, 4);
    posgripper = Tgripper(1:3, 4);

    P(4:6) = rotmat2vec3d(Tgripper(1:3,1:3));   % test: adding orientation gripper
    
    % Compute positions of centers of mass of each joint
    cm01 = A_base * [1 0 0 cm(1,1); 0 1 0 cm(1,2); 0 0 1 cm(1,3); 0 0 0 1];
    cm02 = Tgripper * [1 0 0 cm(2,1); 0 1 0 cm(2,2); 0 0 1 cm(2,3)-z_gripper; 0 0 0 1];       

    % Extract centers of mass positions
    poscm01 = cm01(1:3, 4);
    poscm02 = cm02(1:3, 4);

    % torque calculations
    t2 = cross((poscm02 - pos2), [0 0 -9.81*m(2)]);
    rot2 = T02(1:3,1:3) * [0 0 1]';
    t2_theta = t2*rot2;

    t1 = cross((poscm02 - pos0), [0 0 -9.81*m(2)]) + cross((poscm01 - pos0), [0 0 -9.81*m(1)]);
    rot1 = A_base(1:3,1:3) * [0 0 1]';
    t1_theta = t1*rot1;
    
    rot = A_base(1:3,1:3) * [0 1 0]';
    t0_beta = t1*rot;

    t = [t0_beta t1_theta t2_theta];

    % Vector of joints position
    posx = [0 pos0(1) pos1(1) pos2(1) posgripper(1)];
    posy = [0 pos0(2) pos1(2) pos2(2) posgripper(2)];
    posz = [0 pos0(3) pos1(3) pos2(3) posgripper(3)];

    % Vector of cm positions
    posxx = [poscm01(1) poscm02(1)];
    posyy = [poscm01(2) poscm02(2)];
    poszz = [poscm01(3) poscm02(3)];

%     % Plot the points as red big dots with a line connecting them
%     figure(2);
%     scatter3(posx, posy, posz, 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
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
%     text(posxx, posyy, poszz, ['cm01';'cm02'])
%     
%     % Plot reference frames at each joint
%     scale = 20; % Scale for the quiver arrows
%     plotFrame(A_base, scale);
%     plotFrame(T01, scale);
%     plotFrame(T02, scale);
%     plotFrame(Tgripper, scale);
% 
%     % Plot rot rot1 and rot2
%     plotVector(pos0, rot, scale, 'yellow');
%     plotVector(pos0, rot1, scale, 'magenta');
%     plotVector(pos2, rot2, scale, 'cyan');
end

% Define the buildHD function
function [A] = buildHD(theta, alpha, d, a)
    A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);...
       sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
       0 sin(alpha) cos(alpha) d;...
       0 0 0 1];
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

function plotVector(origin, vector, scale, color)
    quiver3(origin(1), origin(2), origin(3), vector(1)*scale, vector(2)*scale, vector(3)*scale, 'Color', color, 'LineWidth', 1.5);
end
