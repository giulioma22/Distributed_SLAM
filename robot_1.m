clear all;
close all;
clc;

%% ROBOT 1

% addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')
addpath('C:\Users\giuli\OneDrive\Desktop\EIT Trento\Distributed_Systems\MatLab\SLAM_Project\Robotics_Toolbox\src\environment')
load('offlineSlamData.mat');

maxLidarRange = 4;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

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

%Loop for multiple scans
%For ten step iterative visualization
cell_scan=cell(1,N_SCANS);
env.Poses = [1 1 0]';


for idx = 1:N_SCANS

    env.Poses = env.Poses + [0.3 0.2 pi/8]';
    ranges = lidar();
    scan = lidarScan(ranges,lidar.scanAngles);
    cell_scan{idx} = scan; 
    
    %% Send data through TCP/IP

    % Ranges
    data_1 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    fopen(data_1);
    fwrite(data_1, cell_scan{1,idx}.Ranges, 'float')
    fprintf("Sent 1\n");
    fclose(data_1);

    % Angles
    data_2 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    fopen(data_2);
    fwrite(data_2, cell_scan{1,idx}.Angles, 'float')
    fprintf("Sent 2\n");
    fclose(data_2);

    % Cartesian
    data_3 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    fopen(data_3);
    fwrite(data_3, cell_scan{1,idx}.Cartesian(:,1), 'float')
    fprintf("Sent 3\n");
    fclose(data_3);

    data_4 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    fopen(data_4);
    fwrite(data_4, cell_scan{1,idx}.Cartesian(:,2), 'float')
    fprintf("Sent 4\n");
    fclose(data_4);
    
%     if (idx == 5)
%         pause(10)
%     end
    
    % pause(2);

    %% Build SLAM from Lidar data

    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, cell_scan{idx});
    if isScanAccepted
        fprintf('Added scan\n');
    end
    
    figure(1);
    show(slamAlg);
    title({'ROBOT Map','Pose Graph for Initial 10 Scans'});
    hold on
    drawnow

end

% %% Visualise Map built
% figure(1);
% show(slamAlg);
% title({'ROBOT Map','Pose Graph for Initial 10 Scans'});
% drawnow

%%Visualise Environment and robot
% viz = Visualizer2D;
% viz.mapName = 'map';
% attachLidarSensor(viz,lidar)
% viz(env.Poses,ranges);%For one step visualization

