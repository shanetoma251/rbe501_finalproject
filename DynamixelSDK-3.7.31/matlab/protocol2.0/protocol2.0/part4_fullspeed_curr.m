% RBE 501 Team 3 Task 1 Code
clear; clc; 
addpath('/home/zhun/fl_23/ModernRobotics-master/ModernRobotics-master/packages/MATLAB/mr')

%% Screws and Link Info
l0 = 36.08 *10^(-3);%m between earth and j1
l1 = 60.25 *10^(-3); %m between j1 and j2
l2z = 127.85 *10^(-3);%m between j2 and j3
l2x = 24.81 *10^(-3);%m
l3 = 123.91 *10^(-3);%m between j3 and j4
l4 = 141.84 *10^(-3);%m between j4 and ee

% building home configuration
HC = [[0 0 -1 l2x+l3+l4];...
    [0 1 0 0];...
    [1 0 0 l0+l1+l2z];...
    [0 0 0 1]];

% building slist
slist = [[0;0;1;0;0;0],...
    [0;1;0;-(l0+l1);0;0],...
    [0;1;0;-(l0+l1+l2z);0;l2x],...
    [0;1;0;-(l0+l1+l2z);0;(l2x+l3)]];
% target pose
T = [[1,0,0,0.15];...
    [0,1,0,0];...
    [0,0,1,0.05];...
    [0,0,0,1]];
% guess
thetalist0 = [0;0;0;pi/2];
% tolerances
eomg = 0.01;
ev = 0.001;

thetalist = [0; 0; 0; 0];
dthetalist = [0; 44*2*pi/60; 0; 0];
ddthetalist = [0; 0; 0; 0];  % acc and vel are desired to be 0s
g = [0; 0; -9.8];
% applying force
Ftip = [0; 0; 0; 0; 0; 10];    % 10N at z direction of task frame


M01 = [[1, 0, 0, -0.00431]; [0, 1, 0, -0.00101]; [0, 0, 1, 0.02479+l0/2]; [0, 0, 0, 1]];
M12 = [[1, 0, 0, 0.00192]; [0, 1, 0, -0.00065]; [0, 0, 1, 0.09653+l1-0.02479]; [0, 0, 0, 1]];
M23 = [[1, 0, 0, 0.08565+l2x-0.00192]; [0, 1, 0, -0.00050]; [0, 0, 1, 0.00163+l2z-0.09653]; [0, 0, 0, 1]];
M34 = [[1, 0, 0, 0.06252+l3-0.08565]; [0, 1, 0, 0]; [0, 0, 1, 0.00475]; [0, 0, 0, 1]];
M45 = [[0, 0, -1, l4-0.06525]; [0, 1, 0, 0]; [1, 0, 0, 0]; [0, 0, 0, 1]];
% inertia
G1 = diag([31201*10^(-9), 30329*10^(-9), 19507*10^(-9), 0.07618, 0.07618, 0.07618]);
G2 = diag([188396*10^(-9), 186754*10^(-9), 40911*10^(-9), 0.10229, 0.10229, 0.10229]);
G3 = diag([23938*10^(-9), 121558*10^(-9), 132281*10^(-9), 0.08542, 0.08542, 0.08542]);
G4 = diag([122201*10^(-9), 133733*10^(-9), 186759*10^(-9), 0.14413, 0.14413, 0.14413]);
Glist = cat(3, G1, G2, G3, G4);
Mlist = cat(3, M01, M12, M23, M34, M45); 

taulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, ...
                        Ftip, Mlist, Glist, slist)
