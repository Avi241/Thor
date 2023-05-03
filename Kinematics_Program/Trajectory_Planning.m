clear all 
clc

robot = importrobot('arm.urdf');
axes = show(robot);
axes.CameraPositionMode = 'auto';

% wayPoints = [0.2 -0.2 0.02;0.15 0 0.28;0.15 0.05 0.2; 0.15 0.09 0.15;0.1 0.12 0.1; 0.04 0.1 0.2;0.25 0 0.15; 0.2 0.2 0.02];
% wayPointVels = [0 0 0; 0 0.05 0; 0 0.025 -0.025; 0 0.025 -0.025;-0.1 0 0;0.05 0 0;0 -0.1 0;0 0 0];
% 
% % wayPoints = [0.2 -0.2 0.02;
% %              0.25 0 0.15;
% %              0.2 0.2 0.02];
% % wayPointsVels = [0 0 0;
% %                  0 0.1 0;
% %                  0 0 0];
% 
% exampleHelperPlotWaypoints(wayPoints);
% 
% numTotalPoints = size(wayPoints',1)*10;
% waypointTime = 4;
% 
% % Trapezoidal
% trajectory = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);
% 
% % Cubic
% 
% wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
% trajTime = linspace(0,wpTimes(end),numTotalPoints);
% trajectory = cubicpolytraj(wayPoints',wpTimes,trajTime,'VelocityBoundaryCondition',wayPointsVels')


% Create a set of desired waypoints
waypointType = 'simple'; % or 'complex'
switch waypointType
    case 'simple'
        wayPoints = [0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02]; % Alternate set of wayPoints
        wayPointVels = [0 0 0;0 0.1 0;0 0 0];
    case 'complex'
        wayPoints = [0.2 -0.2 0.02;0.15 0 0.28;0.15 0.05 0.2; 0.15 0.09 0.15;0.1 0.12 0.1; 0.04 0.1 0.2;0.25 0 0.15; 0.2 0.2 0.02];
        wayPointVels = [0 0 0; 0 0.05 0; 0 0.025 -0.025; 0 0.025 -0.025;-0.1 0 0;0.05 0 0;0 -0.1 0;0 0 0];
end
exampleHelperPlotWaypoints(wayPoints);
% Create a smooth trajectory from the waypoints
numTotalPoints = size(wayPoints,1)*10;
waypointTime = 4;
trajType = 'trapezoidal'; % or 'trapezoidal'
switch trajType
    case 'trapezoidal'
        trajectory = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);
    case 'cubic'
        wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
        trajTimes = linspace(0,wpTimes(end),numTotalPoints);
        trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                     'VelocityBoundaryCondition',wayPointVels');
end
% Plot trajectory spline and waypoints
hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);


hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);

ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
initialguess = robot.homeConfiguration;
% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
for idx = 1:size(trajectory,2)
    tform = trvec2tform(trajectory(:,idx)');
    configSoln(idx,:) = ik('tool_link',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end
% Visualize robot configurations
title('Robot waypoint tracking visualization')
axis([-0.1 0.4 -0.35 0.35 0 0.35]);

show(robot,configSoln(10,:), 'PreservePlot', false,'Frames','off');










% for idx = 1:size(trajectory,2)
%     show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
%     pause(0.1)
% end
% hold off
