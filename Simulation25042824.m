clc
clear
close all

%% Create reference map
refmap = binaryOccupancyMap(20, 20, 5);

% Walls
for i = 0 : 0.1 : 20
    %external walls
    setOccupancy(refmap, [0.1 i], 1)
    setOccupancy(refmap, [i 0.1], 1)
    if i <= 8 || i >= 12
        setOccupancy(refmap, [19.9 i], 1)
    end
    setOccupancy(refmap, [i 19.9], 1)
end

%Obstacles
for i = 0 : 0.1 : 20
    if  i == 8
        for j = 0 : 0.1 : 5
            setOccupancy(refmap, [j 8], 1)
        end
    end
    if i == 10
        for j = 0 : 0.1 : 5
            setOccupancy(refmap, [j 10], 1)
        end
        for j = 14 : 0.1 : 18
            setOccupancy(refmap, [10 j], 1)
        end
        for j = 2 : 0.1 : 6
            setOccupancy(refmap, [10 j], 1)
        end
    end
    if i == 5
        for j = 8 : 0.1 : 10
            setOccupancy(refmap, [5 j], 1)
        end
    end
    if i == 14
        for j = 4 : 0.1 : 10
            setOccupancy(refmap, [j 14], 1)
        end
    end
    if i == 4
        for j = 14 : 0.1 : 18
            setOccupancy(refmap, [4 j], 1)
        end
        for j = 0 : 0.1 : 1
            setOccupancy(refmap, [j 4], 1)
        end
    end
    if i == 1
        for j = 0 : 0.1 : 4
            setOccupancy(refmap, [1 j], 1)
        end
    end
    if i == 18
        for j = 4 : 0.1 : 10
            setOccupancy(refmap, [j 18], 1)
        end
    end
    if i == 7
        for j = 2 : 0.1 : 6
            setOccupancy(refmap, [7 j], 1)
        end
    end
    if i == 2
        for j = 7 : 0.1 : 10
            setOccupancy(refmap, [j 2], 1)
        end
        for j = 15 : 0.1 : 17
            setOccupancy(refmap, [j 2], 1)
        end
    end
    if i == 6
        for j = 7 : 0.1 : 10
            setOccupancy(refmap, [j 6], 1)
        end
        for j = 15 : 0.1 : 17
            setOccupancy(refmap, [j 6], 1)
        end
    end
    if i == 12
        for j = 12 : 0.1 : 16
            setOccupancy(refmap, [j 12], 1)
        end
        for j = 9 : 0.1 : 12
            setOccupancy(refmap, [12 j], 1)
        end
    end
    if i == 16
        for j = 9 : 0.1 : 12
            setOccupancy(refmap, [16 j], 1)
        end
    end
    if i == 9
        for j = 12 : 0.1 : 16
            setOccupancy(refmap, [j 9], 1)
        end
    end
    if i == 15
        for j = 2 : 0.1 : 6
            setOccupancy(refmap, [15 j], 1)
        end
        for j = 15 : 0.1 : 20
            setOccupancy(refmap, [15 j], 1)
        end
        for j = 15 : 0.1 : 17
            setOccupancy(refmap, [j 15], 1)
        end
    end
    if i == 17
        for j = 2 : 0.1 : 6
            setOccupancy(refmap, [17 j], 1)
        end
        for j = 15 : 0.1 : 20
            setOccupancy(refmap, [17 j], 1)
        end
    end
end

refFigure = figure('Name', 'Indoor Map');
show(refmap)
title('Workshop')

%% Empty map
map = binaryOccupancyMap(20, 20, 10);
mapFigure = figure('Name','Unknown Map');
show(map);

%% RRT
mapInflated = copy(refmap);
inflate(mapInflated, 0.6);

ss = stateSpaceSE2; 
ss.StateBounds = [refmap.XWorldLimits; refmap.YWorldLimits; [-pi pi]];

validator = validatorOccupancyMap(ss);
validator.Map = mapInflated;
validator.ValidationDistance = 0.1;

planner = plannerRRT(ss, validator);
planner.MaxConnectionDistance = 2.0;
planner.MaxIterations = 10000;

start = [2, 2, 0];    % [x, y, theta]
goal = [20, 10, 0];
[pthObj, solnInfo] = plan(planner, start, goal);

show(refmap);
hold on;
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-'); % Show the "Tree"
if solnInfo.IsPathFound
    plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 3); % The final path
end

%% Movement
diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");
controller = controllerPurePursuit('DesiredLinearVelocity', 2,'MaxAngularVelocity',4);

sensor = rangeSensor;
sensor.Range = [0, 5];

%% Route
controller.Waypoints = pthObj.States(:, 1:2);

initPose = [2, 2, pi/2];
goal = [20 10]';
poses(:,1) = initPose';

figure(refFigure);
hold on
plot(pthObj.States(:,1), pthObj.States(:,2), 'o-');
hold off

%% Control
DiffDriveControl(diffDrive,controller,initPose,goal,refmap,map,refFigure,mapFigure,sensor)
