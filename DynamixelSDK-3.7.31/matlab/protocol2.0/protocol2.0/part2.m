

% Path declaration
addpath(pathdef)

% run part2_calculation first to get jointvals_path
% A to C to B in degrees
joint_path = [jointvals_path1, jointvals_path2] .* 180 ./ pi;

travelTime = 20; % Defines the overall travel time
step_time = 0.2; % Each waypoint lasts 0.1s
steps = travelTime/step_time;
index_jump = length(joint_path)/steps; % sampling the path with this index jump/increment


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

robot.writeTime(step_time)
for i = 1:(length(joint_path)-1) % Iterate through waypoints
    index = i*index_jump;
    robot.writeJoints([joint_path(1, index),...
        joint_path(2, index),...
        joint_path(3, index),...
        joint_path(4, index)]); % Write joint values

    tic; % Start timer

    while toc < step_time
        readings =  robot.getJointsReadings(); % Read joint values
        pos = [pos;readings(1,:)];
        vel = [vel;readings(2,:)];
        curr = [curr;readings(3,:)];
    end

end

robot.writeGripper(false);

pause(1);





