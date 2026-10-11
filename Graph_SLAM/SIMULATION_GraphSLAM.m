clc;
clear all;
close all;

% ------------------------------------------------------------------
% Environment: ground-truth landmark positions and path waypoints
% ------------------------------------------------------------------
landmarks = [0.5,  0.5;
             1.5,  0.3;
             2.3,  1.2;
             1.0,  2.0;
             0.2,  1.3;
             2.0, -0.5];
numLandmarks = size(landmarks, 1);

goalPoints = [0,0; 2,0; 2,2; 1,1; 2,0];

% ------------------------------------------------------------------
% Robot/controller parameters
% ------------------------------------------------------------------
v_mps = 0.5;
robotRadius_m = 0.25;

% Odometry/measurement noise std devs (must match getControlAndMeasurement)
sigma_v = 0.02; sigma_w = deg2rad(1);
sigma_r = 0.05; sigma_b = deg2rad(2);

% ------------------------------------------------------------------
% SLAM parameters
% State Vector : x = [x; y; theta; lm1_x; lm1_y; ...; lmN_x; lmN_y]
% Covariance Matrix : P
% Measurement Noise : Q
% ------------------------------------------------------------------
n = 3 + 2*numLandmarks;
x = zeros(n, 1);
x(1:3) = [0, 0.2, 0];

% Robot pose assumed known at start; landmarks start with high uncertainty
% since their true locations haven't been observed yet
P = zeros(n);
P(1:3,1:3) = diag([1e-6, 1e-6, 1e-6]);
largeInitialVariance = 1e6;
for ii = 1:numLandmarks
    jx = 3 + 2*(ii-1) + 1;
    P(jx:jx+1, jx:jx+1) = eye(2)*largeInitialVariance;
end
observedLandmarks = false(numLandmarks, 1);
Q = diag([sigma_r^2, sigma_b^2]);

% ------------------------------------------------------------------
% Simulation loop
% ------------------------------------------------------------------
ii = 1;
lastGoal = x(1:2)';
currentGoal = goalPoints(ii, :);
atFinalGoal = 0;

while(~atFinalGoal)
    estimatedPose = x(1:3)';

    % Simulate one iteration of motion + sensing (known data association)
    [u, z, truePose, correspondence] = getControlAndMeasurement(estimatedPose, v_mps, lastGoal, currentGoal, landmarks);




    

    
    % ---------------- Goal management ----------------
    if(atGoalPoint(truePose, currentGoal))
        if(ii < size(goalPoints, 1))
            lastGoal = currentGoal;
            ii = ii + 1;
            currentGoal = goalPoints(ii, :);
        else
            atFinalGoal = 1;
        end
    end

    % ---------------- Render environment ----------------
    %======================================================================
    clf; hold on;
    title(sprintf('Current Goal Point: %0.2f, %0.2f', currentGoal(1), currentGoal(2)));
    xlim([-1 3.5]); ylim([-1.5 2.5]);
    xlabel('meters'); ylabel('meters');

    % landmarks: true (bright) vs. estimated (gray), stars
    plot(landmarks(:,1), landmarks(:,2), 'r*', 'MarkerSize', 10, 'LineWidth', 1.5);
    seen = find(observedLandmarks);
    if(~isempty(seen))
        lmX = x(3 + 2*(seen-1) + 1);
        lmY = x(3 + 2*(seen-1) + 2);
        plot(lmX, lmY, '*', 'Color', [.7 .7 .7], 'MarkerSize', 10, 'LineWidth', 1.5);
    end

    % robot: true pose (default/bright) vs. estimated pose (gray)
    drawRobot(truePose(1), truePose(2), rad2deg(truePose(3)), robotRadius_m);
    drawRobot(x(1), x(2), rad2deg(x(3)), robotRadius_m, 'gray');

    pause(0.001);
    % End render environment
    %----------------------------------------------------------------------
end

function [atGoal] = atGoalPoint(robotPose, goalPoint)
    goalRadius_m = 0.25;
    dist = sqrt((robotPose(1) - goalPoint(1))^2 + (robotPose(2) - goalPoint(2))^2);
    atGoal = abs(dist) < goalRadius_m;
end

function [wrapped_rad] = wrapToPi(angle_rad)
    % wrap an angle in radians to the range [-pi, pi]
    wrapped_rad = mod(angle_rad + pi, 2*pi) - pi;
end

