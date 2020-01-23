clear all;
clc;

%% ROBOT 1

% addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')
addpath('C:\Users\giuli\OneDrive\Desktop\EIT Trento\Distributed_Systems\MatLab\SLAM_Project\Robotics_Toolbox\src\environment')
load('offlineSlamData.mat');
maxLidarRange = 4;
mapResolution = 20;

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
    cell_scan{1,idx}=scan; 
end

for counter = 1:N_SCANS

    %% Send data through TCP/IP

    % data_01 = zeros(1,200);
    % data_02 = zeros(1,200);

    % for i = 1:100
    %     data_0(i) = cell_scan{1,i}.Ranges;
    %     data_0(i+1) = cell_scan{1,i}.Angles;
    %     data_0(i+2) = cell_scan{1,i}.Cartesian(:,1);
    %     data_0(i+3) = cell_scan{1,i}.Cartesian(:,2);
    % end

    % data_01(1:100) = cell_scan{1,1}.Ranges;
    % data_01(101:200) = cell_scan{1,1}.Angles;
    % data_02(1:100) = cell_scan{1,1}.Cartesian(:,1);
    % data_02(101:200) = cell_scan{1,1}.Cartesian(:,2);
    % 
    % data_all_1 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    % fopen(data_all_1);
    % data_all_1.OutputBufferSize
    % fwrite(data_all_1, data_01, 'float')
    % fprintf("Sent All 1\n");
    % fclose(data_all_1);
    % 
    % clear data_all;
    % 
    % fopen(data_all_2);
    % fwrite(data_all_2, data_02, 'float')
    % fprintf("Sent All 2\n");
    % fclose(data_all_2);


    % Ranges
    data_1 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    fopen(data_1);
    fwrite(data_1, cell_scan{1,counter}.Ranges, 'float')
    fprintf("Sent 1\n");
    fclose(data_1);

    % Angles
    data_2 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    fopen(data_2);
    fwrite(data_2, cell_scan{1,counter}.Angles, 'float')
    fprintf("Sent 2\n");
    fclose(data_2);

    % Cartesian
    data_3 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    fopen(data_3);
    fwrite(data_3, cell_scan{1,counter}.Cartesian(:,1), 'float')
    fprintf("Sent 3\n");
    fclose(data_3);

    data_4 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    fopen(data_4);
    fwrite(data_4, cell_scan{1,counter}.Cartesian(:,2), 'float')
    fprintf("Sent 4\n");
    fclose(data_4);

    % % Count
    % data_5 = tcpip('localhost', 30000, 'NetworkRole', 'client');
    % fopen(data_5);
    % fwrite(data_5, cell_scan{1,1}.Count, 'float')
    % fprintf("Sent 5\n");
    % fclose(data_5);

    %% Build SLAM from Lidar data
    % slamAlg = lidarSLAM(mapResolution, maxLidarRange);
    % slamAlg.LoopClosureThreshold = 210;  
    % slamAlg.LoopClosureSearchRadius = 8;
    % 
    % for i=1:N_SCANS
    %     [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, cell_scan{i});
    %     if isScanAccepted
    %         fprintf('Added scan %d \n', i);
    %     end
    % end

    %% Visualise Map built
    % figure;
    % show(slamAlg);
    % title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

    %%Visualise Environment and robot
    % viz = Visualizer2D;
    % viz.mapName = 'map';
    % attachLidarSensor(viz,lidar)
    % viz(env.Poses,ranges);%For one step visualization
    
    pause(2);
    
end
