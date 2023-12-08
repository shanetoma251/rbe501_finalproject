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
tf = 9.5;
qi = gen_thetalistA;

%generate joint angles for point c
thetalistc_guess = [deg2rad(0); deg2rad(-5); deg2rad(25); deg2rad(-20)];
[gen_thetalistC, success] = IKinSpace(Slist,M,Tsd_C,thetalistc_guess,eomg,ev);
qf = gen_thetalistC;
qf-qi

t2_p1_V_J1 = 3/2*(qf(1)-qi(1))/tf;
t2_p1_V_J2 = 3/2*(qf(2)-qi(2))/tf;
t2_p1_V_J3 = 3/2*(qf(3)-qi(3))/tf;
t2_p1_V_J4 = 3/2*(qf(4)-qi(4))/tf;

t2_p1_V = cat(1,t2_p1_V_J1,t2_p1_V_J2,t2_p1_V_J3,t2_p1_V_J4);

t2_p1_tb = mean((qi(:) - qf(:) + t2_p1_V(:)*tf)./t2_p1_V(:));

N = tf*100;
tpoints1 = linspace(0,tf,N);

for i=1:N
    t = tpoints1(i);
    for j = 1:4
        alpha = (t2_p1_V(j)/t2_p1_tb);
        if t <= t2_p1_tb
            t2_qs_p1(j,i) = qi(j) + alpha/2*t^2;
        elseif (t > t2_p1_tb && t <= (tf-t2_p1_tb))
            t2_qs_p1(j,i) = (qi(j) + qf(j) - t2_p1_V(j)*tf)/2 + t2_p1_V(j)*t;
        elseif t > (tf-t2_p1_tb) && t <= tf
            t2_qs_p1(j,i) =  qf(j) - alpha/2*tf^2 + alpha*tf*t - alpha/2*t^2;
        end
    end
end
t2_qs_p1 = rad2deg(t2_qs_p1);

for i = 1:100
    for j = 1:4
        t2_qs_stop(j,i) = rad2deg(qf(j));
    end
end

tpoints_stop = linspace(tf,10.5,100);

qi = gen_thetalistC; qf = gen_thetalistB;

t2_p2_V_J1 = 3/2*(qf(1)-qi(1))/tf;
t2_p2_V_J2 = 3/2*(qf(2)-qi(2))/tf;
t2_p2_V_J3 = 3/2*(qf(3)-qi(3))/tf;
t2_p2_V_J4 = 3/2*(qf(4)-qi(4))/tf;

t2_p2_V = cat(1,t2_p2_V_J1,t2_p2_V_J2,t2_p2_V_J3,t2_p2_V_J4);

tb = (qi(1) - qf(1) + t2_p2_V_J1*tf)/t2_p2_V_J1;

tpoints2 = linspace(0,tf,N);

for i=1:N
    t = tpoints2(i);
    for j = 1:4
        alpha = (t2_p2_V(j)/tb);
        if t <= tb
            t2_qs_p2(j,i) = qi(j) + (t2_p2_V(j)/(2*tb))*t^2;
        elseif (t > tb && t <= (tf-tb))
            t2_qs_p2(j,i) = (qi(j) + qf(j) - t2_p2_V(j)*tf)/2 + t2_p2_V(j)*t;
        elseif t > (tf-tb) && t <= tf
            t2_qs_p2(j,i) =  qf(j) - alpha/2*tf^2 + alpha*tf*t - alpha/2 * t^2;
        end
    end
end
t2_qs_p2 = rad2deg(t2_qs_p2);

t2_qs = cat(2,t2_qs_p1,t2_qs_stop,t2_qs_p2);

tpoints2 = (tpoints2(:) + 10.5)';
tpoints = cat(2,tpoints1,tpoints_stop,tpoints2);

t2_vels = zeros(4,2000);
for i = 2:2000
    for j=1:4
        t2_vels(j,i) = (t2_qs(j,i) - t2_qs(j,i-1)) / (tpoints(i) - tpoints(i-1));
    end
end
% t2_vels = rad2deg(t2_vels);

t2_accs = zeros(4,1999);
for i = 1:1999
    for j = 1:4
        t2_accs(j,i) = (t2_vels(j,i+1) - t2_vels(j,i)) / (tpoints(i+1) - tpoints(i));
    end
end
% t2_accs = rad2deg(t2_accs);

figure
plot(tpoints,t2_qs(1,:)); hold on;
plot(tpoints,t2_qs(2,:));
plot(tpoints,t2_qs(3,:));
plot(tpoints,t2_qs(4,:));
legend('Joint 1','Joint 2','Joint 3','Joint 4','Location','best'); title('Joint Positions LSPB with Pause');
figure
plot(tpoints,t2_vels(1,:)); hold on;
plot(tpoints,t2_vels(2,:));
plot(tpoints,t2_vels(3,:));
plot(tpoints,t2_vels(4,:));
legend('Joint 1','Joint 2','Joint 3','Joint 4','Location','best'); title('Joint Velocities LSPB with Pause');
figure
plot(tpoints(1:1999),t2_accs(1,:)); hold on;
plot(tpoints(1:1999),t2_accs(2,:));
plot(tpoints(1:1999),t2_accs(3,:));
plot(tpoints(1:1999),t2_accs(4,:));
legend('Joint 1','Joint 2','Joint 3','Joint 4','Location','best'); title('Joint Accelerations LSPB with Pause');

%% Task 3
qA = gen_thetalistA; qB = gen_thetalistB; qC = gen_thetalistC;

% Joint 1 - Constant Velocity LSPB
tf = 20;
t3_V_J1 = 3/2*((qC(1) - qA(1)) + (qB(1) - qC(1)))./tf;
tb = (((qA(1) - qC(1)) + (qC(1) - qB(1))) + t3_V_J1*tf)/t3_V_J1;

N = 2000;
t3_tpoints = linspace(0,tf,N);

t3_vels = zeros(4,N);

for i = 1:N
    t = t3_tpoints(i);
    if t <= tb
        t3_vels(1,i) = t3_V_J1/(tb)*t;
    elseif (t > tb && t <= (tf-tb))
        t3_vels(1,i) = t3_V_J1;
    elseif(t > (tf-tb) && t <= tf)
        t3_vels(1,i) = t3_V_J1/tb*tf - t3_V_J1/tb*t;
    end
end

% Joints 2-4 - Zero Velocity at Waypoint

tf = 10;
t3A_V = zeros(4,1);
t3A_V(2:4) = 3/2*(qC(2:4) - qA(2:4))/tf
t3A_tb = mean((qA(2:4) - qC(2:4) + t3A_V(2:4)*tf) ./ (t3A_V(2:4)))
t3A_tpoints = linspace(0,tf,1000);

t3A_vels = zeros(4,1000);
for i = 1:1000
    for j = 2:4
        t = t3A_tpoints(i);
        if t <= t3A_tb
            t3A_vels(j,i) = t3A_V(j)/(t3A_tb)*t;
        elseif (t > t3A_tb && t <= (tf-t3A_tb))
            t3A_vels(j,i) = t3A_V(j);
        elseif(t > (tf-t3A_tb) && t <= tf)
            t3A_vels(j,i) = t3A_V(j)/t3A_tb*tf - t3A_V(j)/t3A_tb*t;
        end
    end
end

t3B_V = zeros(4,1);
t3B_V(2:4) = 3/2*(qB(2:4) - qC(2:4))/tf;
t3B_tb = mean(((qC(2:4) - qB(2:4)) + t3B_V(2:4)*tf)./t3B_V(2:4));
t3B_tpoints = linspace(0,tf,1000);

t3B_vels = zeros(4,1000);
for i = 1:1000
    for j = 2:4
        t = t3B_tpoints(i);
        if t <= t3B_tb
            t3B_vels(j,i) = t3B_V(j)/(t3B_tb)*t;
        elseif (t > t3B_tb && t <= (tf-t3B_tb))
            t3B_vels(j,i) = t3B_V(j);
        elseif(t > (tf-t3B_tb) && t <= tf)
            t3B_vels(j,i) = t3B_V(j)/t3B_tb*tf - t3B_V(j)/t3B_tb*t;
        end
    end
end

t3B_tpoints = t3B_tpoints(:) + 10;
t3_vels = cat(2,t3A_vels,t3B_vels) + t3_vels;
t3_vels = rad2deg(t3_vels);

t3_qs_1 = cumtrapz(t3_tpoints,t3_vels(1,:)) + rad2deg(qA(1));
t3_qs_2 = cumtrapz(t3_tpoints,t3_vels(2,:)) + rad2deg(qA(2));
t3_qs_3 = cumtrapz(t3_tpoints,t3_vels(3,:)) + rad2deg(qA(3));
t3_qs_4 = cumtrapz(t3_tpoints,t3_vels(4,:)) + rad2deg(qA(4));

int_time = linspace(0,20,size(t3_qs_1,2));

figure
subplot(2,2,1);
plot(t3_tpoints,t3_vels(1,:));
subplot(2,2,2);
plot(t3_tpoints,t3_vels(2,:));
subplot(2,2,3);
plot(t3_tpoints,t3_vels(3,:));
subplot(2,2,4);
plot(t3_tpoints,t3_vels(4,:));

figure
subplot(2,2,1);
plot(int_time,t3_qs_1);
subplot(2,2,2);
plot(int_time,t3_qs_2);
subplot(2,2,3);
plot(int_time,t3_qs_3);
subplot(2,2,4);
plot(int_time,t3_qs_4);







