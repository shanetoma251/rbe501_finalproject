% RBE 501 Team 3 Task 1 Code
addpath(pathdef)
travelTime = 0.5; % Defines the travel time
robot = Robot(); % Creates robot object
% robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(1);

% initialize the logs
vel = zeros([1,4]);
curr = zeros([1,4]);



robot.writeMode('curr position');
robot.writeJoints(0); % Write joints to zero position
pause(2); % Wait for trajectory completion in 2s
robot.writeMode('velocity')

% read from:
% https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
maxVel = 44; % RPM


% converting to deg/s
maxVel_deg = maxVel / 60 * 360;

% velocity waypoints for joint 2
vel_waypoints = [-1, -0.8, -0.6, -0.4] .* maxVel_deg; 

% threshold value
threshold = 210; % unit 0.229 RPM
threshold_deg = 210*0.229 / 60 * 360;

motion_time = 0.3;

tic; % Start timer
t = 0; % Logging elasped time with tic-toc 
del_t = 0;
while del_t < motion_time
    target_vel = vel_waypoints(2); % change the index accordingly to vary the speed
    robot.writeVelocities([0, target_vel, 0, 0])
    readings =  robot.getJointsReadings(); % Read joint values
    vel = [vel;readings(2,:)];
    curr = [curr;readings(3,:)];
    del_t = toc
    t = [t;del_t];
end
        

robot.writeVelocities([0, 0, 0, 0])

% for i = vel_waypoints
%     robot.writeVelocities([0, i, 0, 0])
%     tic; % Start timer
%     while toc < travelTime
%         readings =  robot.getJointsReadings(); % Read joint values
%         vel = [vel;readings(2,:)];
%         curr = [curr;readings(3,:)];
%     end
% end

robot.writeGripper(false);
pause(1);

%% plot
figure
plot(t, curr(:,2))