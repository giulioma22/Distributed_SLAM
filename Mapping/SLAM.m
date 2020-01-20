clear all;
clc;
close all;
%https://www.mathworks.com/help/nav/ug/implement-simultaneous-localization-and-mapping-with-lidar-scans.html

addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')
load('offlineSlamData.mat');
maxLidarRange = 4;
mapResolution = 20;
class(scans{1,1})

%%construct Lidar data object
lidar = MultiRobotLidarSensor;
lidar.sensorOffset = [0,0];
lidar.robotIdx = 1;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-1.5*pi/2,1.5*pi/2,100); % 21 scans from -pi/2 to +pi/2
lidar.maxRange = 4 ;%4 meter

N_SCANS=10;
numRobots = 1;
env = MultiRobotEnv(numRobots);
load exampleMap
env.mapName = 'map';
env.robotRadius = 0.5 ;%*ones(5,1);
env.hasWaypoints = false;
attachLidarSensor(env,lidar);

%%Loop for one scan
% env.Poses = [1 1 pi/2]';
% ranges = lidar();
% scan = lidarScan(ranges,lidar.scanAngles);
% cell_scan=cell(1,1);
% cell_scan{1,1}=scan;


%%Loop for multiple scans
%For ten step iterative visualization
cell_scan=cell(1,N_SCANS);
env.Poses = [1 1 0]';
for idx = 1:N_SCANS
    env.Poses = env.Poses + [0.3 0.2 pi/8]';
    ranges = lidar();
    scan = lidarScan(ranges,lidar.scanAngles);
    cell_scan{1,idx}=scan;    
end


%%Build SLAM from Lidar data
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;


for i=1:N_SCANS
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, cell_scan{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end

%%Visualise Map bulided
figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

%%Visualise Environment and robot
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar)
viz(env.Poses,ranges);%For one step visualization
pause(0.25);
