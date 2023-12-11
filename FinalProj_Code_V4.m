% RBE 501 Team 3 Task 1 Code
clear all; close all; clc;
addpath('mr')

vConst = 1.5;

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

%% Task 2 LSPB with Stop at C - Current Position Control
Tsd_C = [[0 0 -1 0.185];[0 1 0 0];[1 0 0 0.2];[0 0 0 1]];
t0 = 0;
tf = 4.5;
qi = gen_thetalistA;

%generate joint angles for point c
thetalistc_guess = [deg2rad(0); deg2rad(-5); deg2rad(25); deg2rad(-20)];
[gen_thetalistC, success] = IKinSpace(Slist,M,Tsd_C,thetalistc_guess,eomg,ev);
qf = gen_thetalistC;

t2_p1_V_J1 = vConst*(qf(1)-qi(1))/tf;
t2_p1_V_J2 = vConst*(qf(2)-qi(2))/tf;
t2_p1_V_J3 = vConst*(qf(3)-qi(3))/tf;
t2_p1_V_J4 = vConst*(qf(4)-qi(4))/tf;

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

tpoints_stop = linspace(tf,tf+1,100);

qi = gen_thetalistC; qf = gen_thetalistB;

t2_p2_V_J1 = vConst*(qf(1)-qi(1))/tf;
t2_p2_V_J2 = vConst*(qf(2)-qi(2))/tf;
t2_p2_V_J3 = vConst*(qf(3)-qi(3))/tf;
t2_p2_V_J4 = vConst*(qf(4)-qi(4))/tf;

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

tpoints2 = (tpoints2(:) + 5.5)';
tpoints = cat(2,tpoints1,tpoints_stop,tpoints2);

t2_vels = zeros(4,1000);
for i = 2:1000
    for j=1:4
        t2_vels(j,i) = (t2_qs(j,i) - t2_qs(j,i-1)) / (tpoints(i) - tpoints(i-1));
    end
end
% t2_vels = rad2deg(t2_vels);

t2_accs = zeros(4,999);
for i = 1:999
    for j = 1:4
        t2_accs(j,i) = (t2_vels(j,i+1) - t2_vels(j,i)) / (tpoints(i+1) - tpoints(i));
    end
end
% t2_accs = rad2deg(t2_accs);

t2_pos_plot = figure;
plot(tpoints,t2_qs(1,:)); hold on; grid on;
plot(tpoints,t2_qs(2,:));
plot(tpoints,t2_qs(3,:));
plot(tpoints,t2_qs(4,:));
xlabel('Time, $t$ (s)','fontsize',14,Interpreter='latex'); ylabel('Joint Position, $q$ (deg)','fontsize',14,Interpreter='latex')
legend('Joint 1','Joint 2','Joint 3','Joint 4','Location','best'); title('Joint Positions LSPB with Pause','fontsize',16,Interpreter='latex');
t2_vel_plot = figure;
plot(tpoints,t2_vels(1,:)); hold on; grid on;
plot(tpoints,t2_vels(2,:));
plot(tpoints,t2_vels(3,:));
plot(tpoints,t2_vels(4,:));
xlabel('Time, $t$ (s)','fontsize',16,Interpreter='latex'); ylabel('Joint Velocities, $\dot{q}$ ($\frac{\mathrm{deg}}{\mathrm{s}}$)','fontsize',14,Interpreter='latex')
legend('Joint 1','Joint 2','Joint 3','Joint 4','Location','best'); title('Joint Velocities LSPB with Pause','fontsize',16,Interpreter='latex');
t2_acc_plot = figure;
plot(tpoints(1:999),t2_accs(1,:)); hold on; grid on;
plot(tpoints(1:999),t2_accs(2,:));
plot(tpoints(1:999),t2_accs(3,:));
plot(tpoints(1:999),t2_accs(4,:));
xlabel('Time, $t$ (s)','fontsize',14,Interpreter='latex'); ylabel('Joint Accelerations, $\ddot{q}$ ($\frac{\mathrm{deg}}{\mathrm{s}^2}$)','fontsize',14,Interpreter='latex')
legend('Joint 1','Joint 2','Joint 3','Joint 4','Location','best'); title('Joint Accelerations LSPB with Pause','fontsize',16,Interpreter='latex');

%% Task 3 Stitched LSPB with Constant Velocity at C - Vel Control
qA = gen_thetalistA; qB = gen_thetalistB; qC = gen_thetalistC;

% Joint 1 - Constant Velocity LSPB
tf = 10;
t3_V_J1 = vConst*((qC(1) - qA(1)) + (qB(1) - qC(1)))./tf;
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

tf = 5;
t3A_V = zeros(4,1);
t3A_V(2:4) = vConst*(qC(2:4) - qA(2:4))/tf;
t3A_tb = mean((qA(2:4) - qC(2:4) + t3A_V(2:4)*tf) ./ (t3A_V(2:4)));
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
t3B_V(2:4) = vConst*(qB(2:4) - qC(2:4))/tf;
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

t3B_tpoints = t3B_tpoints(:) + tf;
t3_vels = cat(2,t3A_vels,t3B_vels) + t3_vels;
t3_vels = rad2deg(t3_vels);

t3_qs_1 = cumtrapz(t3_tpoints,t3_vels(1,:)) + rad2deg(qA(1));
t3_qs_2 = cumtrapz(t3_tpoints,t3_vels(2,:)) + rad2deg(qA(2));
t3_qs_3 = cumtrapz(t3_tpoints,t3_vels(3,:)) + rad2deg(qA(3));
t3_qs_4 = cumtrapz(t3_tpoints,t3_vels(4,:)) + rad2deg(qA(4));

int_time = linspace(0,10,size(t3_qs_1,2));

t3_vels_plot = figure;
subplot(2,2,1);
plot(t3_tpoints,t3_vels(1,:)); grid on;
subplot(2,2,2);
plot(t3_tpoints,t3_vels(2,:)); grid on;
subplot(2,2,3);
plot(t3_tpoints,t3_vels(3,:)); grid on;
subplot(2,2,4);
plot(t3_tpoints,t3_vels(4,:)); grid on;
t3_vel_axes = axes(t3_vels_plot,'visible','off');
t3_vel_axes.Title.Visible='on';
t3_vel_axes.XLabel.Visible='on';
t3_vel_axes.YLabel.Visible='on';
ylabel(t3_vel_axes,'Joint Velocity, $\dot{q}$ ($\frac{\mathrm{deg}}{\mathrm{s}}$)','fontsize',14,Interpreter='latex');
xlabel(t3_vel_axes,'Time, $t$ (s)','fontsize',14,Interpreter='latex');
title(t3_vel_axes,'Joint Velocities for Stitched LSPB with Constant Velcoity Traversing Waypoint C','fontsize',16,Interpreter='latex');

t3_int_pos_plot = figure;
subplot(2,2,1);
plot(int_time,t3_qs_1); grid on;
subplot(2,2,2);
plot(int_time,t3_qs_2); grid on;
subplot(2,2,3);
plot(int_time,t3_qs_3); grid on;
subplot(2,2,4);
plot(int_time,t3_qs_4); grid on;
t3_int_axes = axes(t3_int_pos_plot,'visible','off');
t3_int_axes.Title.Visible='on';
t3_int_axes.XLabel.Visible='on';
t3_int_axes.YLabel.Visible='on';
ylabel(t3_int_axes,'Calculated Joint Positions, $q$ (deg)','fontsize',14,Interpreter='latex');
xlabel(t3_int_axes,'Time, $t$ (s)','fontsize',14,Interpreter='latex');
title(t3_int_axes,'Joint Positions for Stitched LSPB with Constant Velocity Traversing Waypoint C','fontsize',16,Interpreter='latex');

%% Task 4

l0 = 36.08 *10^(-3);%m between earth and j1
l1 = 60.25 *10^(-3); %m between j1 and j2
l2z = 127.85 *10^(-3);%m between j2 and j3
l2x = 24.81 *10^(-3);%m
l3 = 123.91 *10^(-3);%m between j3 and j4
l4 = 141.84 *10^(-3);%m between j4 and ee

M01 = [[1, 0, 0, -0.00431]; [0, 1, 0, -0.00101]; [0, 0, 1, 0.02479+l0/2]; [0, 0, 0, 1]];
M12 = [[1, 0, 0, 0.00192]; [0, 1, 0, -0.00065]; [0, 0, 1, 0.09653+l1-0.02479]; [0, 0, 0, 1]];
M23 = [[1, 0, 0, 0.08565+l2x-0.00192]; [0, 1, 0, -0.00050]; [0, 0, 1, 0.00163+l2z-0.09653]; [0, 0, 0, 1]];
M34 = [[1, 0, 0, 0.06252+l3-0.08565]; [0, 1, 0, 0]; [0, 0, 1, 0.00475]; [0, 0, 0, 1]];
M45 = [[0, 0, -1, l4-0.06525]; [0, 1, 0, 0]; [1, 0, 0, 0]; [0, 0, 0, 1]];

% inertia
m_motor = 0.087;
m_J2J3 = 0.059;
m_J3J4 = 0.039;
m_gripper = 0.216;

m_l1 = m_motor;
m_l2 = m_motor + m_J2J3;
m_l3 = m_motor + m_J3J4;
m_l4 = m_motor + m_gripper;

G1 = diag([31201*10^(-9), 30329*10^(-9), 19507*10^(-9), m_l1, m_l1, m_l1]);
G2 = diag([188396*10^(-9), 186754*10^(-9), 40911*10^(-9), m_l2, m_l2, m_l2]);
G3 = diag([23938*10^(-9), 121558*10^(-9), 132281*10^(-9), m_l3, m_l3, m_l3]);
G4 = diag([122201*10^(-9), 133733*10^(-9), 186759*10^(-9), m_l4, m_l4, m_l4]);
Glist = cat(3, G1, G2, G3, G4);
Mlist = cat(3, M01, M12, M23, M34, M45);

thetalist = gen_thetalistA;
dthetalist = zeros(4,1);
ddthetalist = zeros(4,1);
g = [0; 0; -9.8];

wrenchlist = [1 1.47 1.38 1.27 0.5 0.295 0.155];

% for i=1:7
%     force = wrenchlist(i);
%     Ftip = [0; 0; 0; 0; 0; -9.81*force];
%     taulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist);
%     disp(taulist)
% end

taulist1 = [0.0195 0.0523 -1.0863 -0.2802]';
taulist2 = [0.0266 0.3059 -1.2591 -0.3185]';
taulist3 = [0.0284 0.3702 -1.3029 -0.3282]';
taulist4 = [0.0212 0.1129 -1.1275 -0.2893]';
taulist5 = [0.0311 0.4686 -1.3700 -0.3431]';
taulist6 = [0.0292 0.4005 -1.3236 -0.3328]';
taulist7 = [0.0269 0.3173 -1.2668 -0.3202]';
taulist8 = [0.0106 -0.2656 -0.8696 -0.2322]';
taulist9 = [0.0063 -0.4207 -0.7638 -0.2087]';
taulist10 = [0.0033 -0.5267 -0.6916 -0.1927]';

taulist = cat(2,taulist1,taulist2,taulist3,taulist4,taulist5,taulist6,taulist7,taulist8,taulist9,taulist10);

currentlist1 = [-1.6779 -351.2714 105.0698 -86.1333]';
currentlist2 = [4.8525 -301.0163 108.4967 -82.2824]';
currentlist3 = [40.1918 -376.2308 117.5688 -79.3286]';
i_1kg = [0 -441.2127 36.1831 -84.6559]';
i_1_47kg = [2.3620 -445.4246 63.5102 -80.8968]';
i_1_38kg = [40.1918 -376.2308 117.5688 -79.3286]';
i_1_27kg = [0.2110 -460.7284 62.5029 -83.8120]';
i_0_5kg = [-3.0885 -426.8798 -99.1979 -92.7884]';
i_0_295kg = [0 -429.6010 -79.5015 -87.7846]';
i_0_155kg = [0.0607 -439.6229 -60.5351 -82.5608]';

i_list = 0.001*cat(2, currentlist1, currentlist2, currentlist3, i_1kg, i_1_47kg, i_1_38kg, i_1_27kg, i_0_5kg, i_0_295kg, i_0_155kg);
% wrenchlist = [1 1.47 1.38 1.27 0.5 0.295 0.155];

figure
plot(i_list',taulist');
legend;

torque_const = taulist./i_list

torque_const_plot = figure;
plot(torque_const(1,:),'.-'); hold on;
plot(torque_const(2,:),'.-');
plot(torque_const(3,:),'.-');
plot(torque_const(4,:),'.-');
axis padded
legend("Joint 1","Joint 2","Joint 3","Joint 4");
title("Torque Constant Calculated from Each Sample",'fontsize',16,Interpreter="latex")
xlabel("Sample Number",'fontsize',14,Interpreter='latex'); ylabel("Torque Constant, $k_T$ ($\frac{\mathrm{Nm}}{\mathrm{A}}$)",'fontsize',14,Interpreter='latex');

torque_const_J1 = torque_const(1,:); torque_const_J1(4) = NaN; torque_const_J1(7) = NaN; torque_const_J1(9:10) = NaN;
kt_mean_J1 = mean(torque_const_J1,"omitmissing")
kt_std_J1 = std(torque_const_J1, "omitmissing")
kt_mean_J2 = mean(torque_const(2,:))
kt_std_J2 = std(torque_const(2,:))
kt_mean_J3 = mean(torque_const(3,:))
kt_std_J3 = std(torque_const(3,:))
kt_mean_J4 = mean(torque_const(4,:))
kt_std_J4 = std(torque_const(4,:))