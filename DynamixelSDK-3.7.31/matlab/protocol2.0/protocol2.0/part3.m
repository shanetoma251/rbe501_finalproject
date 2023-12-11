% RBE 501 Team 3 Task 1 Code
addpath(pathdef)

robot = Robot(); % Creates robot object
% robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(1);

% run part3_calculation.m first to get its worksapce
travelTime = 20; % Defines the overall travel time
step_time = 2/3; % Each waypoint lasts this amount
steps = int16(travelTime/step_time);
index_jump = length(t3_vels)/steps; % sampling the path with this index jump


% initialize the logs
vel = zeros([1,4]);
curr = zeros([1,4]);



robot.writeMode('cp');
robot.writeTime(2); % travelling for 2s
robot.writeJoints(0); % Write joints to zero position
pause(2); % Wait for trajectory completion for 2s

robot.writeTime(2); % travelling for 2s
robot.writeJoints([-44.9828, -5.69539, 24.9715, -19.2761]) % move it to A first
pause(2) % wait for 2s

robot.writeMode('v')
% initialize the logger
pos = zeros([1,4]);
vel = zeros([1,4]);
curr = zeros([1,4]);

robot.writeTime(step_time)
for i = 1:(steps-1) % Iterate through waypoints
    a = tic;
    index = i*index_jump
    robot.writeVelocities([t3_vels(1, index),...
        t3_vels(2, index),...
        t3_vels(3, index),...
        t3_vels(4, index)]); % Write joint values

    tic; % Start timer

    while toc < step_time
        readings =  robot.getJointsReadings(); % Read joint values
        pos = [pos;readings(1,:)];
        vel = [vel;readings(2,:)];
        curr = [curr;readings(3,:)];
    end
    toc(a)
end

robot.writeGripper(false);

pause(1);