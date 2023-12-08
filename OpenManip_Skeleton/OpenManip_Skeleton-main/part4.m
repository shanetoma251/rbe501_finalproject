% code for part 4
addpath(pathdef)
travelTime = 20; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

%% collecting values

robot.writeJoints(0); % Write joints to zero position
pause(2); % Wait for trajectory completion in 2s

q = ones([20,4]); % placeholder, use actual trajectory
pos = zeros([1,4]);
vel = zeros([1,4]);
curr = zeros([1,4]);
for q = baseWayPoints % Iterate through waypoints

    robot.writeJoints([baseWayPoint, 0, 0, 0]); % Write joint values

    tic; % Start timer

    while toc < travelTime
       readings =  robot.getJointsReadings(); % Read joint values
       pos = [pos;readings(1,:)];
       vel = [vel;readings(1,:)];
       curr = [curr;readings(1,:)];
    end

end

robot.writeGripper(false);

pause(1);