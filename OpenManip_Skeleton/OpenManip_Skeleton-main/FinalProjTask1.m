% RBE 501 Team 3 Task 1 Code
clear all; close all; clc;
addpath('mr')

%% Screws and Link Info
l1 = 36.076e-3; l2 = 60.10e-3; l3 = 128e-3; l4 = 24e-3; l5 = 124e-3; l6 = 133.4e-3;

S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -(l1+l2) 0 0]';
S3 = [0 1 0 -(l1+l2+l3) 0 l4]';
S4 = [0 1 0 -(l1+l2+l3) 0 (l4+l5)]';
Slist = cat(2,S1,S2,S3,S4);

M = [[0 0 -1 (l4+l5+l6)]; ...
    [0 1 0 0]; ...
    [1 0 0 (l1+l2+l3)]; ...
    [0 0 0 1]]; % Home Config of robot

A = [0.185 -0.185 0.185]'; % Locations to reach
B = [0.185 0.170 0.030]';

tA = atan2(A(2),A(1)) % Find base angle of rotation, assuming straight approach from J1
tB = atan2(B(2),B(1))

Tsd_A = [[0 -sin(tA) -cos(tA); 0 cos(tA) -sin(tA); 1 0 0] A; 0 0 0 1] % Desired transform
Tsd_B = [[0 -sin(tB) -cos(tB); 0 cos(tB) -sin(tB); 1 0 0] B; 0 0 0 1]

%% Plotting Derived Axes
xhatM = cat(2, (M(1:3,4)), (10e-3*M(1:3,1) + M(1:3,4)));
yhatM = cat(2, (M(1:3,4)), (10e-3*M(1:3,2) + M(1:3,4)));
zhatM = cat(2, (M(1:3,4)), (10e-3*M(1:3,3) + M(1:3,4)));
plot3(xhatM(1,:),xhatM(2,:),xhatM(3,:),'Color','r','LineWidth',2);
hold on; grid on; xlim([0 0.3]); ylim([-0.3 0.3]); zlim([0 0.3]); xlabel('x'); ylabel('y'); zlabel('z');
plot3(yhatM(1,:),yhatM(2,:),yhatM(3,:),'Color','g','LineWidth',2);
plot3(zhatM(1,:),zhatM(2,:),zhatM(3,:),'Color','b','LineWidth',2);

xhatA = cat(2, (A(1:3)), (10e-3*Tsd_A(1:3,1) + A(1:3)));
yhatA = cat(2, (A(1:3)), (10e-3*Tsd_A(1:3,2) + A(1:3)));
zhatA = cat(2, (A(1:3)), (10e-3*Tsd_A(1:3,3) + A(1:3)));
plot3(xhatA(1,:),xhatA(2,:),xhatA(3,:),'Color','r','LineWidth',2);
plot3(yhatA(1,:),yhatA(2,:),yhatA(3,:),'Color','g','LineWidth',2);
plot3(zhatA(1,:),zhatA(2,:),zhatA(3,:),'Color','b','LineWidth',2);

xhatB = cat(2, (B(1:3)), (10e-3*Tsd_B(1:3,1) + B(1:3)));
yhatB = cat(2, (B(1:3)), (10e-3*Tsd_B(1:3,2) + B(1:3)));
zhatB = cat(2, (B(1:3)), (10e-3*Tsd_B(1:3,3) + B(1:3)));
plot3(xhatB(1,:),xhatB(2,:),xhatB(3,:),'Color','r','LineWidth',2);
plot3(yhatB(1,:),yhatB(2,:),yhatB(3,:),'Color','g','LineWidth',2);
plot3(zhatB(1,:),zhatB(2,:),zhatB(3,:),'Color','b','LineWidth',2);

%% IK
guess_thetalistA = [deg2rad(-45) deg2rad(-5) deg2rad(24) deg2rad(-19)]';
guess_thetalistB = [deg2rad(42) deg2rad(53) deg2rad(36) deg2rad(-89)]';

for i = (tA+0.001):-0.0001:(tA-0.001)
    eomg = 0.001;
    ev = 0.0001;
    Tsd_A = [[0 -sin(i) -cos(i); 0 cos(i) -sin(i); 1 0 0] A; 0 0 0 1];
    [gen_thetalistA, success] = IKinSpace(Slist,M,Tsd_A,guess_thetalistA,eomg,ev);
    if success == 1
        disp(['Generated Theta Values for A: ', num2str(rad2deg(gen_thetalistA(:)'))]);
        break;
    else
        % disp('IK for A Failed');
    end
end

for j = (tB-0.001):0.0001:(tB+0.001)
    eomg = 0.001;
    ev = 0.0001;
    Tsd_B = [[0 -sin(j) -cos(j); 0 cos(j) -sin(j); 1 0 0] B; 0 0 0 1];
    [gen_thetalistB, success] = IKinSpace(Slist,M,Tsd_B,guess_thetalistB,eomg,ev);
    if success == 1
        disp(['Generated Theta Values for B: ', num2str(rad2deg(gen_thetalistB(:)'))]);
        break;
    else
        % disp('IK for B Failed');
    end
end