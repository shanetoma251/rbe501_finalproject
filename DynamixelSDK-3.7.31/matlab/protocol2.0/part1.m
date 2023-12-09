% RBE 501 Team 3 Task 1 Code
addpath(pathdef)
travelTime = 0.5; % Defines the travel time
robot = Robot(); % Creates robot object
% robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(1);
%% collecting values
robot.writeMode('position');
robot.writeJoints(0); % Write joints to zero position
pause(2); % Wait for trajectory completion in 2s
robot.writeMode('velocity')

% read from:
% https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#velocity-limit44
maxVel = 0.229; % RPM,

% converting to deg/s
maxVel_rad = maxVel / 60 * 360;

% velocity waypoints for joint 2
vel_waypoints = [-1, -0.8, +0.6, 0.4] .* maxVel_rad; 

% initialize the logs
vel = zeros([1,4]);
curr = zeros([1,4]);
robot.writeVelocities([0, 210, 0, 0])
for i = vel_waypoints

    robot.writeVelocities([0, i, 0, 0])
    tic; % Start timer
    
    while toc < travelTime
        readings =  robot.getJointsReadings(); % Read joint values
        vel = [vel;readings(2,:)];
        curr = [curr;readings(3,:)];
    end
end
robot.writeGripper(false);
vel
curr
pause(1);