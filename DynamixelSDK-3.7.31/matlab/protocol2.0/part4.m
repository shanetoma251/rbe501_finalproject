%% Setup robot
addpath(pathdef)
% initialize the robot, set step time, and enable motor holding at last
% pose
robot = Robot(); % Creates robot object
robot.writeMotorState(true); % Write position mode

robot.writeTime(2); % travelling for 2s
robot.writeJoints(0); % Write joints to zero position
pause(2); % Wait for trajectory completion for 2s

robot.writeTime(2); % travelling for 2s
robot.writeJoints([-44.9828, -5.69539, 24.9715, -19.2761]) % move it to A first
pause(2) % wait for 2s

% initialize the logger
pos = zeros([1,4]);
vel = zeros([1,4]);
curr = zeros([1,4]);

% loop end signifier
loop = 1;
while loop == 1 % stop this process with ctrl+C
    
    tic; % Start timer
    pause(2) % let it stay at A for 5s for first question in part A

    fprintf('The external wrench can be applied now.')

    while toc < travelTime
       readings =  robot.getJointsReadings(); % Read joint values
       pos = [pos;readings(1,:)];
       vel = [vel;readings(2,:)];
       curr = [curr;readings(3,:)];
    end

end

robot.writeGripper(false);

pause(1);
%%
plot(curr)