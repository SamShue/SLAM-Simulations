clc;
clear all;
close all;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'common'));

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

% ------------------------------------------------------------------
% Simulation parameters shared with getControlAndMeasurement
% ------------------------------------------------------------------
simParams.landmarks = landmarks;
simParams.model     = 'icr';         % kinematic model: 'icr' or 'simple'
simParams.dt_s      = 0.1;           % time between iterations (s)
simParams.sigma_v   = 0.02;          % linear velocity noise std dev (m/s)
simParams.sigma_w   = deg2rad(1);    % angular velocity noise std dev (rad/s)
simParams.sigma_r   = 0.05;          % range measurement noise std dev (m)
simParams.sigma_b   = deg2rad(2);    % bearing measurement noise std dev (rad)

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
Q = diag([simParams.sigma_r^2, simParams.sigma_b^2]);

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
    [u, z, truePose, correspondence] = getControlAndMeasurement(estimatedPose, v_mps, lastGoal, currentGoal, simParams);

    % ---------------- EKF predict ----------------
    theta = x(3);
    v = u(1); w = u(2); dt = u(3);

    x(1) = x(1) + v*cos(theta)*dt;
    x(2) = x(2) + v*sin(theta)*dt;
    x(3) = wrapToPi(x(3) + w*dt);

    Fx_pose = [1, 0, -v*sin(theta)*dt;
               0, 1,  v*cos(theta)*dt;
               0, 0,  1];
    G = eye(n);
    G(1:3,1:3) = Fx_pose;

    % map velocity noise into pose-space process noise
    V = [cos(theta)*dt, 0;
         sin(theta)*dt, 0;
         0,             dt];
    M = diag([simParams.sigma_v^2, simParams.sigma_w^2]);
    R_motion = zeros(n);
    R_motion(1:3,1:3) = V*M*V';

    P = G*P*G' + R_motion;

    % ---------------- EKF correct ----------------
    j = correspondence;
    jx = 3 + 2*(j-1) + 1;
    jy = jx + 1;

    if(~observedLandmarks(j))
        % First sighting: seed the landmark estimate from the current pose/measurement
        x(jx) = x(1) + z(1)*cos(z(2) + x(3));
        x(jy) = x(2) + z(1)*sin(z(2) + x(3));
        observedLandmarks(j) = true;
    end

    delta = [x(jx) - x(1); x(jy) - x(2)];
    q = delta'*delta;
    zhat = [sqrt(q); wrapToPi(atan2(delta(2), delta(1)) - x(3))];

    F_j = zeros(5, n);
    F_j(1:3, 1:3) = eye(3);
    F_j(4, jx) = 1;
    F_j(5, jy) = 1;

    H_low = (1/q)*[-sqrt(q)*delta(1), -sqrt(q)*delta(2), 0,     sqrt(q)*delta(1), sqrt(q)*delta(2); ...
                     delta(2),         -delta(1),        -q,   -delta(2),         delta(1)];
    H = H_low*F_j;

    S = H*P*H' + Q;
    K = P*H'/S;

    innovation = z(:) - zhat;
    innovation(2) = wrapToPi(innovation(2));

    x = x + K*innovation;
    x(3) = wrapToPi(x(3));
    P = (eye(n) - K*H)*P;

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

