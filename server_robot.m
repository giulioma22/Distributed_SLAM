
%% SERVER

clear all;
clc;

% addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')
addpath('C:\Users\giuli\OneDrive\Desktop\EIT Trento\Distributed_Systems\MatLab\SLAM_Project\Robotics_Toolbox\src\environment')
load('offlineSlamData.mat');

maxLidarRange = 4;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

while (1)
    
%     clear all;
%     clc;

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
    
    env.Poses = [1 1 0]';
    
    %% Read data from robots
    
%     data_0 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');  % 'Timeout', 10
%     fopen(data_0);
%     data_read_0 = fread(data_0, data_0.BytesAvailable/4, 'float');
%     fclose(data_0);
%     
%     data_ranges = data_read_0(1:100);
%     data_angles = data_read_0(101:200);
%     
%     clear data_0;
%     
%     fopen(data_0);
%     data_read_0 = fread(data_0, data_0.BytesAvailable/4, 'float');
%     fclose(data_0);
%     
%     data_cart1 = data_read_0(1:100);
%     data_cart2 = data_read_0(101:200);
    
    data_1 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');  % 'Timeout', 10
    fopen(data_1);
    while (data_1.BytesAvailable == 0)
    end
    data_read_1 = fread(data_1, data_1.BytesAvailable/4, 'float');
    fclose(data_1);
    
    data_2 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
    fopen(data_2);
    while (data_2.BytesAvailable == 0)
    end
    data_read_2 = fread(data_2, data_2.BytesAvailable/4, 'float');
    fclose(data_2);
    
    data_3 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
    fopen(data_3);
    while (data_3.BytesAvailable == 0)
    end
    data_read_3 = fread(data_3, data_3.BytesAvailable/4, 'float');
    fclose(data_3);
    
    data_4 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
    fopen(data_4);
    while (data_4.BytesAvailable == 0)
    end
    data_read_4 = fread(data_4, data_4.BytesAvailable/4, 'float');
    fclose(data_4);
    
%     data_5 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
%     fopen(data_5);
%     data_read_5 = fread(data_5, data_5.BytesAvailable/4, 'float');
%     fclose(data_5);
    
    %% Plot
%     slamAlg = lidarSLAM(mapResolution, maxLidarRange);
%     slamAlg.LoopClosureThreshold = 210;  
%     slamAlg.LoopClosureSearchRadius = 8;
    
    ranges = lidar();
    scan = lidarScan(ranges,lidar.scanAngles);
    
%     scan.Ranges = data_ranges;
%     scan.Angles = data_angles;
%     scan.Cartesian(:,1) = data_cart1;
%     scan.Cartesian(:,2) = data_cart2;
    
    scan.Ranges = data_read_1;
    scan.Angles = data_read_2;
    scan.Cartesian(:,1) = data_read_3;
    scan.Cartesian(:,2) = data_read_4;
%     scan.Count = data_read_5;
    
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
    if isScanAccepted
        fprintf('Added scan\n');
    end

    figure(1);
    show(slamAlg);
    title({'Map of the Environment','Pose Graph for Initial 10 Scans'});
    hold on
    drawnow
    

%     plot(data_read);
%     hold on
%     drawnow
    
%     load('offlineSlamData.mat');
%     maxLidarRange = 4;
%     mapResolution = 20;
%     class(scans{1,1})
% 
%     %%Build SLAM from Lidar data
%     slamAlg = lidarSLAM(mapResolution, maxLidarRange);
%     slamAlg.LoopClosureThreshold = 210;  
%     slamAlg.LoopClosureSearchRadius = 8;
% 
%     for i=1:N_SCANS
%         [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, cell_scan{i});
%         if isScanAccepted
%             fprintf('Added scan %d \n', i);
%         end
%     end
% 
%     %Visualise Map built
%     figure;
%     show(slamAlg);
%     title({'Map of the Environment','Pose Graph for Initial 10 Scans'});    

end