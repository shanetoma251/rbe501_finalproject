% -------------------------------------------------------------------
%  Generated by MATLAB on 10-Dec-2023 13:45:25
%  MATLAB version: 23.2.0.2428915 (R2023b) Update 4
% -------------------------------------------------------------------
saveVarsMat = load('part1_60speed.mat');

curr = ...
  [0 0 0 0;
   0 -107.6 -102.21999999999998 -18.83;
   0 -231.33999999999997 -164.08999999999997 -32.279999999999994;
   0 -242.09999999999997 -142.57 -21.52;
   0 -207.13 -153.32999999999998 -16.139999999999997;
   0 -207.13 -150.64 -26.9;
   0 -207.13 -150.64 -16.139999999999997;
   0 -207.13 -150.64 -24.209999999999997];

maxVel = 44;

maxVel_deg = 264;

readings = ...
  [-0.087890625 2.109375 2.724609375 0.52734375;
   0 0 0 0;
   0 -207.13 -150.64 -24.209999999999997];

robot = saveVarsMat.robot; % <1x1 Robot> unsupported class

target_vel = -158.4;

threshold = 210;

threshold_deg = 288.54;

travelTime = 0.5;

vel = ...
  [0 0 0 0;
   0 2.748 0 0;
   0 8.244 6.87 1.374;
   0 9.618 8.244 1.374;
   0 6.87 2.748 0;
   0 1.374 0 0;
   0 0 0 0;
   0 0 0 0];

vel_waypoints = [-264 -211.20000000000002 -158.4 -105.60000000000001];

clear saveVarsMat;