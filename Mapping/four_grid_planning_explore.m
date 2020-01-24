clear all;
clc;
close all;

%% Simulation setup
addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')

% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);
% Sample time and time array
sampleTime = 0.3;              % Sample time [s]
tVec = 0:sampleTime:45;        % Time array
% Initial conditions
% initPose = [2;2;0];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));     % Pose matrix
% Load map
close all
load exampleMap
% Create lidar 
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-1.5*pi/2,1.5*pi/2,151);
%lidar.scanAngles = linspace(-pi/2,pi/2,101);
lidar.maxRange = 5;%5 orig
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
% waypoints = [1 1 ; 
%              3 3;
%              9 3;
%              3 9;
%              9 9;
%              9 3];
         
         
% waypoints = [1 1 ; 
%              2 3;
%              4 4;
%              5,5;];
          
%%Waypoint random generator
A=[1,2,3,4];%Grid Sections

WAYPOINT_LEN=5;
offset=0.75;
grid_1=[0+offset 6;0+offset 6-offset ];
grid_2=[0+offset 6;6 12-offset];
grid_3=[6+offset 12-offset;0+offset 6-offset];
grid_4=[6+offset 12-offset;6+offset 12-offset];
waypoint_1 = zeros(WAYPOINT_LEN,2);
k=1;
D=0;
inflate_map= copy(map) ;
inflate(inflate_map,0.8);
%show(inflate_map);

GRID_ID=randi(length(A));
if GRID_ID==1
   grid=grid_1;
elseif GRID_ID==2
   grid=grid_2;
elseif GRID_ID==3
   grid=grid_3;
elseif GRID_ID==4
   grid=grid_4;
else
    disp("ILLEGAL MAP");
end
     
    
        
    
while k<=WAYPOINT_LEN
    random_x = grid(1,1) + (grid(1,2)-grid(1,1)) .* rand(1,1);
    random_y = grid(2,1) + (grid(2,2)-grid(2,1)) .* rand(1,1);
    
%     random_x = grid_1_ax + (grid_1_bx-grid_1_ax) .* rand(1,1);
%     random_y = grid_1_ax + (grid_1_bx-grid_1_ax) .* rand(1,1);
    %Map collision check
    occupied = checkOccupancy(inflate_map,[random_x random_y]);
    if 1%~occupied
        waypoint_1(k,1)=random_x ;
        waypoint_1(k,2)=random_y ;
        %disp(waypoint_1)
        if k<3
            k=k+1;
        else
            D = pdist2(waypoint_1(k,:),waypoint_1(k-1,:));
            disp(D);
            if D > 1.8
                k=k+1;
            end
        end
        
    end
    
end

waypoints=waypoint_1;
pose(:,1) = [waypoints(1,1);waypoints(1,2);0];
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
vfh.MinTurningRadius = 0.1;%0.25 orig

N_SCANS=numel(tVec) ;        
cell_scan=cell(1,N_SCANS);
%env.Poses = [3.8 3.3 4.7124]';
Temp_poses=[1 1 0 ]';
f1 = figure;
title('Lidar Mapping'); 
%%Build SLAM from Lidar data
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;
f1 = figure;
title('Lidar Mapping');    
%% Simulation loop
r = rateControl(1/sampleTime);
wpx=2;
for idx = 2:numel(tVec) 
    
    if(wpx > length(waypoints))
        disp("WayPoints Completed")
        break;
    end
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
    scan = lidarScan(ranges,lidar.scanAngles);
    cell_scan{1,idx}=scan; 
    % Run the path following and obstacle avoidance algorithms
    controller.Waypoints = waypoints(wpx,:);
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
    
    %%stop condition
    dist=sqrt((pose(1,idx-1)-waypoints(wpx,1))^2 + (pose(2,idx-1)-waypoints(wpx,2))^2);
    if(dist<1.85)
       wpx=wpx+1;
       disp("############Waypoint Reached########");
    end
    %Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    set(0, 'CurrentFigure', f1);
    show(slamAlg);
    drawnow;
    hold on;
        
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    waitfor(r);
    

end