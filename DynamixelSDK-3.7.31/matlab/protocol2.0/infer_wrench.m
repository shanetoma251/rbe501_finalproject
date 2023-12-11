%% Setup calculations
addpath(pathdef)
addpath '/home/zhun/fl_23/ModernRobotics-master/ModernRobotics-master/packages/MATLAB/mr'

% initialize the logger
pos = zeros([1,4]);
vel = zeros([1,4]);
curr = zeros([1,4]);

% torque constants 
linear_fit_tau_const = [0.7512, -0.1681, -7.2635, 3.3639];
mean_tau_const = [0.8380, -0.2411, -8.7486, 3.4149];

% loop end signifier
loop = 1;

% dimensions in m
l1 = 36.076e-3; l2 = 60.10e-3; l3 = 128e-3; l4 = 24e-3; l5 = 124e-3; l6 = 133.4e-3;

% Body screwlist
Blist = [[1; 0; 0; 0; l6+l5+l4; 0],...
    [0; 1; 0; -l6-l5-l4; 0; -l3],...
    [0; 1; 0; -l6-l5; 0; 0],...
    [0; 1; 0; -l6; 0; 0]];
% Space screwlist
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -(l1+l2) 0 0]';
S3 = [0 1 0 -(l1+l2+l3) 0 l4]';
S4 = [0 1 0 -(l1+l2+l3) 0 (l4+l5)]';
Slist = cat(2,S1,S2,S3,S4);

% These separate dimensions are used to form M list
l1 = 60.25 *10^(-3); %m between j1 and j2
l2z = 127.85 *10^(-3);%m between j2 and j3
l2x = 24.81 *10^(-3);%m
l3 = 123.91 *10^(-3);%m between j3 and j4
l4 = 141.84 *10^(-3);%m between j4 and ee

% M list
M01 = [[1, 0, 0, -0.00431]; [0, 1, 0, -0.00101]; [0, 0, 1, 0.02479+l0/2]; [0, 0, 0, 1]];
M12 = [[1, 0, 0, 0.00192]; [0, 1, 0, -0.00065]; [0, 0, 1, 0.09653+l1-0.02479]; [0, 0, 0, 1]];
M23 = [[1, 0, 0, 0.08565+l2x-0.00192]; [0, 1, 0, -0.00050]; [0, 0, 1, 0.00163+l2z-0.09653]; [0, 0, 0, 1]];
M34 = [[1, 0, 0, 0.06252+l3-0.08565]; [0, 1, 0, 0]; [0, 0, 1, 0.00475]; [0, 0, 0, 1]];
M45 = [[0, 0, -1, l4-0.06525]; [0, 1, 0, 0]; [1, 0, 0, 0]; [0, 0, 0, 1]];
Mlist = cat(3, M01, M12, M23, M34, M45);

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

dthetalist = zeros(4,1);
ddthetalist = zeros(4,1);
g = [0; 0; -9.8];

%% Driving the robot
% initialize the robot, set step time, and enable motor holding at last
% pose
robot = Robot(); % Creates robot object
robot.writeMotorState(true); % Write position mode

robot.writeTime(2); % travelling for 2s
robot.writeJoints(0); % Write joints to zero position
pause(2); % Wait for trajectory completion for 2s
while loop == 1 % stop this process with ctrl+C
    
    tic; % Start timer


    while 1
       readings =  robot.getJointsReadings(); % Read joint values
       pos = [pos;readings(1,:)];
       vel = [vel;readings(2,:)];
       curr = [curr;readings(3,:)];

       % print current values
       fprintf('Joint current at this time in mA: \n')
       readings(3,:)

       % joint torques in kg m
       tau1 = linear_fit_tau_const.*readings(3,:) ./1000 ./9.81;
       tau2 = mean_tau_const.*readings(3,:) ./1000 ./9.81; 
       thetaList = readings(1,:) .*pi ./180;

       % gravitation-induced torque
       grav = GravityForces(thetaList,g,Mlist,Glist,Slist);
       
       % calculate body Jacobian
       Jb = JacobianBody(Blist, thetaList);


       % to calculate e.e. wrenchs from joint torques, this equation is
       % used
       % tau = transpose(J_b)*Fb+grav
       % Fb = transpose(J_b)\(tau - grav)
       Fb1 = transpose(Jb)\(tau1'-grav);
       Fb2 = transpose(Jb)\(tau2'-grav);
       fprintf("Estimation of body wrench(kg) with linear fit constant: \n")
       Fb1
       fprintf("Estimation of body wrench(kg) with mean constant: \n")
       Fb2
       pause(0.5)
    end

end

robot.writeGripper(false);

pause(1);
%%
plot(curr)
legend('I(joint1)', 'I(joint2)', 'I(joint3)', 'I(joint4)')
