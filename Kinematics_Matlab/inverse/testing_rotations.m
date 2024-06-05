clc
clear all

x = [1 0 0 0];
x = rot(x,'+y');
x = rot(x,'+y');
rotations(1,:) = x;
x = rot(x,'+z');
rotations(2,:) = x;
x = rot(x,'+z');
rotations(3,:) = x;
x = rot(x,'+z');
rotations(4,:) = x;
x = rot(x,'+y');
rotations(5,:) = x;
x = rot(x,'+z');
x = rot(x,'+z');
rotations(6,:) = x;
x = rot(x,'-z');
x = rot(x,'-z');
x = rot(x,'+x');
rotations(7,:) = x;
x = rot(x,'+z');
x = rot(x,'+z');
rotations(8,:) = x;
x = rot(x,'-z');
x = rot(x,'-z');
x = rot(x,'+x');
rotations(9,:) = x;
x = rot(x,'+z');
x = rot(x,'+z');
rotations(10,:) = x;
extra = x;
x = rot(x,'-z');
x = rot(x,'-z');
x = rot(x,'+x');
rotations(11,:) = x;
x = rot(x,'+z');
x = rot(x,'+z');
rotations(12,:) = x;

rotations(13,:) = rot(extra,'-y15');
rotations(14,:) = rot(extra,'-y20');
rotations(15,:) = rot(extra,'-y30');



function result = rot(q1, string)

   if strcmp('+x', string)
       q2 = [1/sqrt(2), 1/sqrt(2), 0, 0];
   elseif strcmp('-x', string)
       q2 = [1/sqrt(2), -1/sqrt(2), 0, 0];
   elseif strcmp('+y', string)
       q2 = [1/sqrt(2), 0, 1/sqrt(2), 0];
   elseif strcmp('-y', string)
       q2 = [1/sqrt(2), 0, -1/sqrt(2), 0];
   elseif strcmp('+z', string)
       q2 = [1/sqrt(2), 0, 0, 1/sqrt(2)];
   elseif strcmp('-z', string)
       q2 = [1/sqrt(2), 0, 0, -1/sqrt(2)];
   elseif strcmp('-y15', string)
       q2 = [cos(pi/24), 0, -sin(-pi/24),0 ];
   elseif strcmp('-y20', string)
       q2 = [cos(pi/18), 0, -sin(-pi/18),0 ];
   elseif strcmp('-y30', string)
       q2 = [cos(pi/12), 0, -sin(-pi/12),0 ];
   end

% Perform quaternion multiplication to get the product
w = q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
x = q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
y = q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
z = q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1);

% Store the elements of the resulting quaternion in an array
result= [w, x, y, z];

end