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
initPose = [1;1;0];
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;
% Load map
close all
load exampleMap
% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-1.5*pi/2,1.5*pi/2,101);
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
waypoints = [1 1 ; 
             3 3;
             3 9;
             9 9;
             9 3];
% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
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
for idx = 2:numel(tVec) 
    
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
    waitfor(r);
end