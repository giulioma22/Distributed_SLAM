clear all;
clc;

addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')
lidar = MultiRobotLidarSensor;
lidar.robotIdx = 3;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,21); % 21 scans from -pi/2 to +pi/2
lidar.maxRange = 4 %4 meter


numRobots = 5;
env = MultiRobotEnv(numRobots);
load exampleMap
env.mapName = 'map';
env.robotRadius = 0.5*ones(5,1);
env.hasWaypoints = false;
attachLidarSensor(env,lidar);


env.Poses = [1 1 0;2 2 0;3 3 0;4 4 0;5 5 0]';
ranges = lidar();
scan = lidarScan(ranges,lidar.scanAngles);
%disp(class(scan))

plot(lidar.scanAngles,ranges,'bo-');
xlabel('Angle [rad]')
ylabel('Range [m]')


allRanges = cell(1,numRobots);
allRanges{3} = ranges;
env(1:numRobots,env.Poses,allRanges)