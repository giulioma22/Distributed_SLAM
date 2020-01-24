clear all;
clc;
close all;

%% NEW ROBOT 1

%% Simulation setup

% addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')
addpath('C:\Users\giuli\OneDrive\Desktop\EIT Trento\Distributed_Systems\MatLab\SLAM_Project\Robotics_Toolbox\src\environment')

% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
% Sample time and time array
sampleTime = 0.5;              % Sample time [s]
tVec = 0:sampleTime:45;        % Time array
% Initial conditions
% initPose = [2;2;0];            % Initial pose (x y theta)
% initPose = [1;1;0];
pose = zeros(3,numel(tVec));   % Pose matrix
% pose(:,1) = initPose;
% Load map
close all
load exampleMap
% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-1.5*pi/2,1.5*pi/2,150);
%lidar.scanAngles = linspace(-pi/2,pi/2,101);
lidar.maxRange = 5;     %5 orig
% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Grids parameters

% Points per area
WAYPOINT_LEN = 5;
offset = 1.5;

% Grid dimensions
grid_1 = [0+offset 6-offset; 0+offset 6-offset];
grid_2 = [0+offset 6-offset; 6+offset 12-offset];
grid_3 = [6+offset 12-offset; 6+offset 12-offset];
grid_4 = [6+offset 12-offset; 0+offset 6-offset];

% gridsArray = [grid_1; grid_2; grid_3; grid_4];
wayPointsArray = [];
numberOfGrids = 4;
   
% Sample random point for ALL areas
for j = 1:numberOfGrids
    
    waypoint_1 = zeros(WAYPOINT_LEN,2);
    k = 1;
    D = 0;
    
    if j == 1
       grid = grid_1;
    elseif j == 2
       grid = grid_2;
    elseif j == 3
       grid = grid_3;
    elseif j == 4
       grid = grid_4;
    end
    
    while k <= WAYPOINT_LEN
        
        random_x = grid(1,1) + (grid(1,2)-grid(1,1)) .* rand(1,1);
        random_y = grid(2,1) + (grid(2,2)-grid(2,1)) .* rand(1,1);

        waypoint_1(k,1) = random_x;
        waypoint_1(k,2) = random_y;
        %disp(waypoint_1)
        if k < 3
            k = k+1;
        else
            D = pdist2(waypoint_1(k,:), waypoint_1(k-1,:));
            if D > 1.8
                k = k+1;
            end
        end   
    end
    
    wayPointsArray = [wayPointsArray waypoint_1];
    
end

%% Current grid

currGrid = 3;

% Select waypoints of current grid
waypoints = wayPointsArray(:, (currGrid*2)-1:currGrid*2);
pose(:,1) = [waypoints(1,1); waypoints(1,2); 0];

% First goal is point 2
wayPoint_idx = 2;

% Areas already seen
table = [currGrid];

%% Planning Parameters

% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints(1,:);
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;
% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 3];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [5 10];
vfh.RobotRadius = L;
vfh.SafetyDistance = L;
vfh.MinTurningRadius = 0.15;%0.25 orig

N_SCANS=numel(tVec) ;        
cell_scan=cell(1,N_SCANS);
%env.Poses = [3.8 3.3 4.7124]';
Temp_poses=[1 1 0 ]'; 
%%Build SLAM from Lidar data
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;
f1 = figure;
title('Lidar Mapping');   

%% Simulation loop

SLAM_completed = false;
idx = 1;
minDistance = 1.8;
otherRobotDone = false;

while (~SLAM_completed)

    idx = idx + 1;
    
    if (idx > numel(tVec))
        break
    end
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
    scan = lidarScan(ranges,lidar.scanAngles);
    cell_scan{1,idx}=scan; 
    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    %Lidar Mapping
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, cell_scan{idx});
    if isScanAccepted
        fprintf('Added scan %d \n', idx);
    end
    
    %Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    set(0, 'CurrentFigure', f1);
    show(slamAlg);
    drawnow;
    hold on;
        
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    
    % Check if waypoint was reached
    distance = sqrt((pose(1,idx) - waypoints(wayPoint_idx,1))^2 + (pose(2,idx) - waypoints(wayPoint_idx,2))^2);
    
    % Check if reached waypoint
    if (distance < minDistance)
        fprintf("\n\nReached Waypoint\n\n");
        % If last point in area
        if (wayPoint_idx == WAYPOINT_LEN)
            if (otherRobotDone)
                break
            end
            % New waypoint (only if 0)
            for i = 2:numberOfGrids
                % If NOT checked
                if (int8(ismember(i, table)) == 0)  % False (not in table)
                    isCellOccupied = i;
                    break
                % If ALL already checked
                elseif (i == numberOfGrids && int8(ismember(i, table)) == 1)
                    isCellOccupied = numberOfGrids + 1;
                end
            end
        else
           % Go to next random point
           isCellOccupied = -1;
           wayPoint_idx = wayPoint_idx + 1;
           controller.Waypoints = waypoints(wayPoint_idx,:);
        end
    else
        isCellOccupied = -1;
    end
    
    if (SLAM_completed)
        continue
    end
        
%% COMMUNICATION W/ SERVER

    waiting_for_answer = true;

    while (waiting_for_answer && otherRobotDone == false)

        % Exit loop iff no pending response
        waiting_for_answer = false;
        if (isCellOccupied > 1 && isCellOccupied <= numberOfGrids)
           waiting_for_answer = true; 
        end

        %% Receive server flag
        server_call = tcpip('localhost', 22220, 'NetworkRole', 'client');
        
        % Wait for connection w/ server
        isOpened = false;
        while(~isOpened)
            isOpened = true;
            try
                fopen(server_call);
                while (server_call.BytesAvailable == 0)
                end
                server_flag = fread(server_call, server_call.BytesAvailable, 'int8');
                fclose(server_call);
            catch
                isOpened = false;
                fprintf("Reconnecting...\n");
            end
        end

        %% Answer depending on server flag

        % 0) Other robot is done
        if (server_flag == -2) 
            fprintf("Robot 2 DONE)\n");
            otherRobotDone = true;
            break
        end
        
        % 1) Free to make request
        if (server_flag == -1)          
            fprintf("No server request (-1)\n");
            message = isCellOccupied;
            % 1.1) No request to make
            if (message == -1)
                fprintf("No request (-1)\n");
            % 1.2) Make request
            else
                if (isCellOccupied > numberOfGrids)
                    message = -2;
                    waiting_for_answer = false;
                    fprintf("Turning off... (%d)\n", message);
                else
                   fprintf("Request sent (%d)\n", message); 
                end
            end
            
        % 2) Answer to previous request
        elseif (server_flag == 0 || server_flag == 1)       
            fprintf("Answer to previous request (%d)\n", server_flag);
            % Add value to table (either scanned already or to be scanned)
            table = [table, isCellOccupied];
            % 2.1) Not yet scanned
            if (server_flag == 0)   
                message = -1;
                waiting_for_answer = false;
                wayPoint_idx = 1;
                waypoints = wayPointsArray(:, (isCellOccupied*2)-1:isCellOccupied*2);
                controller.Waypoints = waypoints(wayPoint_idx,:);
                fprintf("No new request (%d)\n", message);
            % 2.2) Already scanned
            else                    
                isCellOccupied = isCellOccupied + 1;
                % 2.2.1) Break if all waypoints were checked -- EXIT Condition
                if (isCellOccupied > numberOfGrids)
                    message = -2;
                    waiting_for_answer = false;
                    SLAM_completed = true;
                    fprintf("Turning off... (%d)\n", message);
                % 2.2.2) Request next waypoint available
                else
                    message = isCellOccupied;
                    fprintf("Request sent (%d)\n", message); 
                end
            end
            
        % 3) Forwarded request from other client
        else                            
            fprintf("Request from other client (%d)\n", server_flag);
            message = int8(ismember(server_flag, table)); 
            fprintf("Answer to client 1 (%d)\n", message);
        end

        % Send message to server
        data_send = tcpip('localhost', 22222, 'NetworkRole', 'client');
        fopen(data_send);
        fwrite(data_send, message, 'int8');
        fclose(data_send);

    %     pause(1);

    end
    
end

fprintf("\n\n- - - SLAM COMPLETED - - -\n\n");