%% import robot model

clc 
clear all
close all

lbr = importrobot("C:\Users\marco\Desktop\Collision_detetion\LRM_ARM_testing\urdf\LRM_ARM_testing2.urdf");
lbr.DataFormat = 'row';
gripper = 'End_effector';

field = linspace(-deg2rad(100), deg2rad(100),3);
counter = 1;
configuration = zeros(729,6);
for i = 1:3
    for j = 1:3
        for k = 1:3
            for l = 1:3
                for m = 1:3
                    for n = 1:3
                        configuration(counter,1:7) = [counter field(i) field(j)/2 field(k) field(l) field(m) field(n)/2];
                        counter = counter + 1;
                    end
                end
            end
        end
    end
end

counter = 1;
counter2 = 1;
for i = 1:height(configuration)
    close all
    config = configuration(i, 2:7);
    [isSelfColliding,selfSeparationDist,selfWitnessPts] = checkCollision(lbr,config, "SkippedSelfCollisions","adjacent");
    tform = getTransform(lbr, config, 'End_effector');
    TCP = tform(1:3,4);

    if isSelfColliding || min(selfSeparationDist(:,end)) < 0.03 || selfSeparationDist(7,8) < 0.16 || TCP(1) < 0.15
        discarded(counter,:) = configuration(i,:);
        counter = counter + 1;
    else
        goodconf(counter2,:) = configuration(i,:);
        counter2 = counter2 + 1;
    end
end

%% EXPORT QINTERP AS TXTFILE
length = height(goodconf);
length1 = round(length)/3;
length2 = length1;
length3 = length - 2*length1;

export =round(goodconf(1:length1,2:7)*10*57.2958,0);
writematrix(export,'poses1.txt');
export =round(goodconf(length1+1:2*length1,2:7)*10*57.2958,0);
writematrix(export,'poses2.txt');
export =round(goodconf(2*length1+1:end,2:7)*10*57.2958,0);
writematrix(export,'poses3.txt');

%% EXPORT GOOD POSES AS .MAT
save("goodconf.mat", "goodconf")