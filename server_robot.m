clear all;
close all;
clc;

%% SERVER

% addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')
addpath('C:\Users\giuli\OneDrive\Desktop\EIT Trento\Distributed_Systems\MatLab\SLAM_Project\Robotics_Toolbox\src\environment')

maxLidarRange = 4;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

counter = 0;

while (1)
    
    %% Read data from robots
    
    counter = counter + 1;
    
    % Ranges
    data_1 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');  % 'Timeout', 10
    fopen(data_1);
    while (data_1.BytesAvailable == 0)
    end
    data_read_1 = fread(data_1, data_1.BytesAvailable/4, 'float');
    fclose(data_1);
    
    % Angles
    data_2 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
    fopen(data_2);
    while (data_2.BytesAvailable == 0)
    end
    data_read_2 = fread(data_2, data_2.BytesAvailable/4, 'float');
    fclose(data_2);
    
    % Cartesian col 1
    data_3 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
    fopen(data_3);
    while (data_3.BytesAvailable == 0)
    end
    data_read_3 = fread(data_3, data_3.BytesAvailable/4, 'float');
    fclose(data_3);
    
    % Cartesian col 2
    data_4 = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
    fopen(data_4);
    while (data_4.BytesAvailable == 0)
    end
    data_read_4 = fread(data_4, data_4.BytesAvailable/4, 'float');
    fclose(data_4);
    
    %% Plot
    
    % Initialize empty scan
    scan = lidarScan(0,0);
    
    % Create scan w/ values from robot
    scan.Ranges = data_read_1;
    scan.Angles = data_read_2;
    scan.Cartesian(:,1) = data_read_3;
    scan.Cartesian(:,2) = data_read_4;
    
    % Add to slamAlg for plotting
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
    if isScanAccepted
        fprintf('Added scan\n');
    end

    % Plot SLAM in real-time
    figure(1);
    show(slamAlg);
    title({'SERVER Map','Pose Graph for Initial 10 Scans'});
    hold on
    drawnow
    
%     if (counter == 5)
%         break
%     end
 
%     %Visualise Map built
%     figure;
%     show(slamAlg);
%     title({'Map of the Environment','Pose Graph for Initial 10 Scans'});    

end