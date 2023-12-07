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

%% Task 2
%desired waypoint c
Tsd_C = [[0 0 -1 0.185];[0 1 0 0];[1 0 0 0.2];[0 0 0 1]];
t0 = 0;
tf = 10;
tb = 2;
qi = gen_thetalistA;

%generate joint angles for point c
thetalistc_guess = [deg2rad(0); deg2rad(-5); deg2rad(25); deg2rad(-20)];
[gen_thetalistC, success] = IKinSpace(Slist,M,Tsd_C,thetalistc_guess,eomg,ev);
qf = gen_thetalistC;
qf-qi

V_J1 = 3/2*(qf(1)-qi(1))/tf;
V_J2 = 3/2*(qf(2)-qi(2))/tf;
V_J3 = 3/2*(qf(3)-qi(3))/tf;
V_J4 = 3/2*(qf(4)-qi(4))/tf;

V = cat(1,V_J1,V_J2,V_J3,V_J4);

tb1 = (qi(1) - qf(1) + V_J1*tf)/V_J1;
tb2 = (qi(2) - qf(2) + V_J2*tf)/V_J2;
tb3 = (qi(3) - qf(3) + V_J3*tf)/V_J3;
tb4 = (qi(4) - qf(4) + V_J4*tf)/V_J4;

tb = tb1; % approximately true

N = 1000;
tpoints1 = linspace(0,10,N);

for i=1:N
    for j = 1:4
        t = tpoints1(i);
        if t <= tb
            jointvals_path1(j,i) = qi(j) + (V(j)/(2*tb))*t^2;
        elseif (t > tb && t <= (tf-tb))
            jointvals_path1(j,i) = (qi(j) + qf(j) - V(j)*tf)/2 + V(j)*t;
        elseif t > (tf-tb) && t <= tf
            alpha = (V(j)/tb);
            jointvals_path1(j,i) =  qf(j) - alpha/2*tf^2 + alpha*tf*t - alpha/2 * t^2;
        end
    end
end

figure
subplot(2,2,1);
plot(tpoints1,jointvals_path1(1,:));
subplot(2,2,2);
plot(tpoints1,jointvals_path1(2,:));
subplot(2,2,3);
plot(tpoints1,jointvals_path1(3,:));
subplot(2,2,4);
plot(tpoints1,jointvals_path1(4,:));

qi = gen_thetalistC; qf = gen_thetalistB;

V_J1 = 3/2*(qf(1)-qi(1))/tf;
V_J2 = 3/2*(qf(2)-qi(2))/tf;
V_J3 = 3/2*(qf(3)-qi(3))/tf;
V_J4 = 3/2*(qf(4)-qi(4))/tf;

V = cat(1,V_J1,V_J2,V_J3,V_J4);

tb1 = (qi(1) - qf(1) + V_J1*tf)/V_J1;
tb2 = (qi(2) - qf(2) + V_J2*tf)/V_J2;
tb3 = (qi(3) - qf(3) + V_J3*tf)/V_J3;
tb4 = (qi(4) - qf(4) + V_J4*tf)/V_J4;

tb = tb1;

N = 1000;
tpoints2 = linspace(0,10,N);

for i=1:N
    for j = 1:4
        t = tpoints2(i);
        if t <= tb
            jointvals_path2(j,i) = qi(j) + (V(j)/(2*tb))*t^2;
        elseif (t > tb && t <= (tf-tb))
            jointvals_path2(j,i) = (qi(j) + qf(j) - V(j)*tf)/2 + V(j)*t;
        elseif t > (tf-tb) && t <= tf
            alpha = (V(j)/tb);
            jointvals_path2(j,i) =  qf(j) - alpha/2*tf^2 + alpha*tf*t - alpha/2 * t^2;
        end
    end
end

tpoints2 = tpoints2(:) + 10;

figure
subplot(2,2,1);
plot(tpoints2,jointvals_path2(1,:));
subplot(2,2,2);
plot(tpoints2,jointvals_path2(2,:));
subplot(2,2,3);
plot(tpoints2,jointvals_path2(3,:));
subplot(2,2,4);
plot(tpoints2,jointvals_path2(4,:));

figure
subplot(2,2,1);
plot(tpoints1,jointvals_path1(1,:)); hold on;
plot(tpoints2,jointvals_path2(1,:));
subplot(2,2,2);
plot(tpoints1,jointvals_path1(2,:)); hold on;
plot(tpoints2,jointvals_path2(2,:));
subplot(2,2,3);
plot(tpoints1,jointvals_path1(3,:)); hold on;
plot(tpoints2,jointvals_path2(3,:));
subplot(2,2,4);
plot(tpoints1,jointvals_path1(4,:)); hold on;
plot(tpoints2,jointvals_path2(4,:));

% %% Task 3
% qA = gen_thetalistA; qB = gen_thetalistB; qC = gen_thetalistC;
% 
% tf = 20;
% 
% V_task3(:) = 3/2*((qC(:) - qA(:)) + (qB(:) - qC(:)))./tf;
% tb = ( ((qA(1) - qC(1)) + (qC(1) - qB(1))) + V_task3(1)*tf)/V_task3(1);
% 
% N = 1000;
% tpoints_task3 = linspace(0,tf,N);
% 
% joint_vels = zeros(4,N);
% 
% for i = 1:N
%     for j = 1:4
%         t = tpoints_task3(i);
%         if t <= tb
%             joint_vels(j,i) = V_task3(j)/(tb)*t;
%         elseif (t > tb && t <= (tf-tb))
%             joint_vels(j,i) = V_task3(j);
%         elseif(t > (tf-tb) && t <= tf)
%             joint_vels(j,i) = V_task3(j)/tb*tf - V_task3(j)/tb*t;
%         end
%     end
% end
% 
% joint_pos_task3_J1 = cumtrapz(tpoints_task3,joint_vels(1,:)) + qA(1);
% joint_pos_task3_J2 = cumtrapz(tpoints_task3,joint_vels(2,:)) + qA(2);
% joint_pos_task3_J3 = cumtrapz(tpoints_task3,joint_vels(3,:)) + qA(3);
% joint_pos_task3_J4 = cumtrapz(tpoints_task3,joint_vels(4,:)) + qA(4);
% 
% int_time = linspace(0,20,size(joint_pos_task3_J1,2));
% 
% % for i = 1:N-1
% %     for j = 1:4
% %         joint_pos_task3(j,i) = joint_vels(j,i+1)*(tpoints_task3(i+1)-tpoints_task3(i));
% %     end
% % end
% 
% figure
% subplot(2,2,1);
% plot(tpoints_task3,joint_vels(1,:));
% subplot(2,2,2);
% plot(tpoints_task3,joint_vels(2,:));
% subplot(2,2,3);
% plot(tpoints_task3,joint_vels(3,:));
% subplot(2,2,4);
% plot(tpoints_task3,joint_vels(4,:));
% 
% figure
% subplot(2,2,1);
% plot(int_time,joint_pos_task3_J1);
% subplot(2,2,2);
% plot(int_time,joint_pos_task3_J2);
% subplot(2,2,3);
% plot(int_time,joint_pos_task3_J3);
% subplot(2,2,4);
% plot(int_time,joint_pos_task3_J4);
% 






