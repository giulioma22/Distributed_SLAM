clear all;
clc;
addpath('D:\UniTn\distributed automation\Project\mathworks-robotics-mobile-robotics-simulation-toolbox-2.2-2-g7066fa0\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0\src\environment')

lidar = LidarSensor

load exampleMap
lidar.mapName = 'map';
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,11); % 11 scans from -pi/2 to +pi/2
lidar.maxRange = 4;

pose = [2; 3; pi];
ranges = lidar(pose)
plot(lidar.scanAngles,ranges,'o-');
xlabel('Angle [rad]')
ylabel('Range [m]')

viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar)
%viz(pose,ranges) %For one step visualization

%For ten step iterative visualization
for idx = 1:100
    pose = pose + [0; 0; pi/8];
    ranges = lidar(pose);
    viz(pose,ranges)
    pause(0.25)
end