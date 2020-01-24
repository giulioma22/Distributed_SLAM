clear all;
clc;
close all;

%% NEW ROBOT 2

%% Simulation setup

% addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')
addpath('C:\Users\giuli\OneDrive\Desktop\EIT Trento\Distributed_Systems\MatLab\SLAM_Project\Robotics_Toolbox\src\environment')

% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
% Sample time and time array
sampleTime = 0.3;              % Sample time [s]
tVec = 0:sampleTime:45;        % Time array
% Initial conditions
% initPose = [2;2;0];            % Initial pose (x y theta)
initPose = [12;12;pi];
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;
% Load map
close all
load exampleMap
% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-1.5*pi/2,1.5*pi/2,100);
%lidar.scanAngles = linspace(-pi/2,pi/2,101);
lidar.maxRange = 8;%5 orig
% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);
%% Path planning and following
% Create waypoints
% waypoints = [initPose(1:2)'; 
%              2 10;
%              11 8;
%              8 2];
waypoints = [12 12; 
             3 3;
             3 9;
             9 9;
             9 3];
         
% Bool map of checked points
numPoints = 5;
table = [1 4];
wayPoint_idx = 4;
% boolMap = [1 0 0 1 0];
% wayPoint_idx = 4;

% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints(wayPoint_idx,:);
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
% r = rateControl(1/sampleTime);
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
    if (distance < 0.6)
        fprintf("\n\nREACHED Waypoint\n\n");
        if (otherRobotDone)
            break
        end
        % New waypoint (only if 0)
        for i = 2:numPoints
            % If NOT checked
            if (int8(ismember(i, table)) == 0)
                isCellOccupied = i;
                break
            % If ALL already checked
            elseif (i == numPoints && int8(ismember(i, table)) == 1)
                isCellOccupied = numPoints + 1;
            end
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
        if (isCellOccupied > 1 && isCellOccupied <= numel(waypoints))
           waiting_for_answer = true; 
        end

        %% Receive server flag
        server_call = tcpip('localhost', 22220, 'NetworkRole', 'client');
%         fopen(server_call);
%         while (server_call.BytesAvailable == 0)
%         end
%         server_flag = fread(server_call, server_call.BytesAvailable, 'int8');
%         fclose(server_call);
        
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
            fprintf("Robot 1 DONE\n");
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
                if (isCellOccupied > numPoints)
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
                wayPoint_idx = isCellOccupied;
                controller.Waypoints = waypoints(isCellOccupied,:);
                fprintf("No new request (%d)\n", message);
            % 2.2) Already scanned
            else                    
                isCellOccupied = isCellOccupied + 1;
                % 2.2.1) Break if all waypoints were checked -- EXIT Condition
                if (isCellOccupied > numPoints)
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

        data_send = tcpip('localhost', 22222, 'NetworkRole', 'client');
        fopen(data_send);
        fwrite(data_send, message, 'int8');
        fclose(data_send);

    %     pause(1);

    end
    
end

fprintf("\n\n- - - SLAM COMPLETED - - -\n\n");