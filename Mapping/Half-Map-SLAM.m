clear all;
clc;
close all;
%https://www.mathworks.com/help/nav/ug/implement-simultaneous-localization-and-mapping-with-lidar-scans.html

addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')
load('offlineSlamData.mat');
maxLidarRange = 4;
mapResolution = 20;
class(scans{1,1});

%%construct Lidar data object
lidar = MultiRobotLidarSensor;
lidar.sensorOffset = [0,0];
lidar.robotIdx = 1;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-1.5*pi/2,1.5*pi/2,100); % 21 scans from -pi/2 to +pi/2
lidar.maxRange = 4 ;%4 meter


numRobots = 1;
env = MultiRobotEnv(numRobots);
load exampleMap
env.mapName = 'map';
env.robotRadius = 0.5 ;%*ones(5,1);
env.hasWaypoints = false;
attachLidarSensor(env,lidar);


%%Build SLAM from Lidar data
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 4;

%%Loop for multiple scans
%For ten step iterative visualization
%%Visualise Environment and robot
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);


store_Poses =[1 1 0;1.3 1.2 0.3927;1.6 1.4 0.7854;1.9 1.6 1.1781;2.2 1.8 1.5708;
            2.5 2 1.9635;2.8 2.2 2.3562;3.1 2.4 2.7489;3.4 2.6 3.1416;3.7 2.8 3.5343;
            4 3 3.9270;4.3 3.2 4.3197;4.6 3.4 4.7124;
            3.8 3.3 4.7124;3.8 3.9 5.1051;3.8 4.5 5.4978; 3.8 5.1 5.8905;3.8 5.7 6.2832;3.8 6.3 6.6759;3.8 6.9 7.0686;3.8 7.5 7.4613;
            3.8 8.1 7.8540;3.8 8.7 8.2467;3.8 9.3 8.6394;3.8 9.9 9.0321;3.8 10.5 9.4248;3.8 11.1 9.8175;3.8 11.7 10.2102]';

N_SCANS=14;        
cell_scan=cell(1,N_SCANS);
env.Poses = [3.8 3.3 4.7124]';
Temp_poses=[1 1 0 ]';
f1 = figure;
title('Lidar Mapping');    
for i=1:length(store_Poses)
   
    %Lidar generate and store
    %env.Poses = env.Poses + [0.0 0.6 pi/8]';
    env.Poses=store_Poses(:,i);
    Temp_poses = [Temp_poses env.Poses];
    ranges = lidar();
    scan = lidarScan(ranges,lidar.scanAngles);
    cell_scan{1,i}=scan;   

    %Lidar Mapping
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, cell_scan{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end

    %%Visualise Map bulided
    set(0, 'CurrentFigure', f1);
    show(slamAlg);
    drawnow;
    hold on;

    %viz(env.Poses,ranges);%For one step visualization
    viz(env.Poses,ranges);
    pause(0.25);
    drawnow;
    hold on;
    

end