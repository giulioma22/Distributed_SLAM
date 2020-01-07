clear all;
clc;

load('offlineSlamData.mat');

maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

% for i=1:10
%     [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
%     if isScanAccepted
%         fprintf('Added scan %d \n', i);
%     end
% end

%% Real-time SLAM

firstTimeLCDetected = false;

figure;
for i=10:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        continue;
    end
    % visualize the first detected loop closure, if you want to see the
    % complete map building process, remove the if condition below
%     if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph, 'IDs', 'off'); 
        hold off;
        firstTimeLCDetected = true;
        drawnow
%     end
end
title('Map with Loop Closures');

figure
show(slamAlg);
title({'Final Map and Robot Trajectory'});

%% Occupancy Grid

[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

figure; 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map');
