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

% The robot's own odometry (e.g. wheel encoders) is an imperfect *reading*
% of its actual motion, separate from the real-world disturbances already
% baked into the true motion above. This extra noise is what the robot
% dead-reckons with, and it's the main thing that makes dead reckoning
% drift away from the truth over time.
simParams.sigma_v_odom = 0.15;        % extra linear odometry noise std dev (m/s)
simParams.sigma_w_odom = deg2rad(1);  % extra angular odometry noise std dev (rad/s)

% ------------------------------------------------------------------
% SLAM parameters
%
% Unlike a filter (e.g. EKF SLAM), GraphSLAM doesn't correct a running
% estimate every iteration. Instead it collects the *entire* history of
% odometry and landmark measurements, builds a "graph" of constraints
% between every robot pose and every landmark, and then solves for the
% most likely trajectory + map all at once (a batch optimization).
%
% So during the simulation loop below we only do dead reckoning
% (integrate noisy odometry with no correction) and log everything
% needed to build that graph afterward:
%   poseHistory     - dead-reckoned pose estimate at every time step
%   controlHistory  - noisy odometry reading [v, w, dt] used at every time step
%   measurementLog  - every landmark observation [poseIdx, landmarkId, range, bearing]
%   truePoseHistory - the robot's true pose at every time step (for plotting only)
% ------------------------------------------------------------------
poseHistory = [0, 0.2, 0];
controlHistory = zeros(0, 3);
measurementLog = zeros(0, 4);
truePoseHistory = poseHistory;

% Landmarks are given a naive initial guess the first time they're seen
% (triangulated from the current, drifting, dead-reckoned pose).
% GraphSLAM will refine every pose and landmark afterward.
landmarkEstimates = zeros(numLandmarks, 2);
observedLandmarks = false(numLandmarks, 1);
Q = diag([simParams.sigma_r^2, simParams.sigma_b^2]);

% ------------------------------------------------------------------
% Simulation loop
% ------------------------------------------------------------------
ii = 1;
lastGoal = poseHistory(1,1:2);
currentGoal = goalPoints(ii, :);
atFinalGoal = 0;

while(~atFinalGoal)
    estimatedPose = poseHistory(end, :);

    % Simulate one iteration of motion + sensing (known data association)
    [u, z, truePose, correspondence] = getControlAndMeasurement(estimatedPose, v_mps, lastGoal, currentGoal, simParams);

    % ---------------- Dead reckoning ----------------
    % The robot doesn't have access to the "true" noisy control u above
    % (that reflects whatever actually happened, including real-world
    % disturbances); it only has its own further-imperfect odometry
    % *reading* of that control (e.g. wheel-encoder error). This extra
    % odometry noise is what GraphSLAM will need to reason about.
    v = u(1) + simParams.sigma_v_odom*randn;
    w = u(2) + simParams.sigma_w_odom*randn;
    dt = u(3); theta = estimatedPose(3);
    newPose = [estimatedPose(1) + v*cos(theta)*dt, ...
               estimatedPose(2) + v*sin(theta)*dt, ...
               wrapToPi(theta + w*dt)];
    poseHistory(end+1, :) = newPose;
    controlHistory(end+1, :) = [v, w, dt];
    truePoseHistory(end+1, :) = truePose;

    % ---------------- Log measurement ----------------
    j = correspondence;
    if(~observedLandmarks(j))
        % First sighting: seed a naive landmark estimate from the current pose/measurement
        landmarkEstimates(j, :) = newPose(1:2) + z(1)*[cos(z(2) + newPose(3)), sin(z(2) + newPose(3))];
        observedLandmarks(j) = true;
    end
    measurementLog(end+1, :) = [size(poseHistory,1), j, z(1), z(2)];

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

    % landmarks: true (bright) vs. naive estimate (gray), stars
    plot(landmarks(:,1), landmarks(:,2), 'r*', 'MarkerSize', 10, 'LineWidth', 1.5);
    seen = find(observedLandmarks);
    if(~isempty(seen))
        plot(landmarkEstimates(seen,1), landmarkEstimates(seen,2), '*', 'Color', [.7 .7 .7], 'MarkerSize', 10, 'LineWidth', 1.5);
    end

    % robot: true pose (default/bright) vs. dead-reckoned pose (gray)
    drawRobot(truePose(1), truePose(2), rad2deg(truePose(3)), robotRadius_m);
    drawRobot(newPose(1), newPose(2), rad2deg(newPose(3)), robotRadius_m, 'gray');

    pause(0.001);
    % End render environment
    %----------------------------------------------------------------------
end

% ------------------------------------------------------------------
% GraphSLAM back-end
% Now that the full trajectory has been driven, build the graph from
% every logged motion and measurement constraint and solve it.
% ------------------------------------------------------------------
[optimizedPoses, optimizedLandmarks] = graphSLAM(poseHistory, controlHistory, measurementLog, landmarkEstimates, observedLandmarks, simParams, Q);

% ---------------- Render final result ----------------
clf; hold on;
title('GraphSLAM result: true vs. dead-reckoned vs. optimized');
xlim([-1 3.5]); ylim([-1.5 2.5]);
xlabel('meters'); ylabel('meters');

plot(landmarks(:,1), landmarks(:,2), 'r*', 'MarkerSize', 10, 'LineWidth', 1.5);
plot(optimizedLandmarks(observedLandmarks,1), optimizedLandmarks(observedLandmarks,2), '*', 'Color', [0 .6 0], 'MarkerSize', 10, 'LineWidth', 1.5);

plot(truePoseHistory(:,1), truePoseHistory(:,2), 'b-', 'LineWidth', 1.5);
plot(poseHistory(:,1), poseHistory(:,2), '--', 'Color', [.7 .7 .7], 'LineWidth', 1.5);
plot(optimizedPoses(:,1), optimizedPoses(:,2), '-', 'Color', [0 .6 0], 'LineWidth', 1.5);

legend('true landmarks', 'optimized landmarks', 'true path', 'dead-reckoned path', 'optimized path', 'Location', 'northeastoutside');

function [atGoal] = atGoalPoint(robotPose, goalPoint)
    goalRadius_m = 0.25;
    dist = sqrt((robotPose(1) - goalPoint(1))^2 + (robotPose(2) - goalPoint(2))^2);
    atGoal = abs(dist) < goalRadius_m;
end

function [wrapped_rad] = wrapToPi(angle_rad)
    % wrap an angle in radians to the range [-pi, pi]
    wrapped_rad = mod(angle_rad + pi, 2*pi) - pi;
end

function [poses, landmarks] = graphSLAM(poseHistory, controlHistory, measurementLog, landmarkEstimates, observedLandmarks, simParams, Q)
%GRAPHSLAM Batch-optimize a pose/landmark graph with Gauss-Newton
%   Builds the information matrix H and information vector b describing
%   every motion constraint (between consecutive poses) and every
%   measurement constraint (between a pose and the landmark it observed),
%   then solves H*dx = -b for the correction that makes the whole graph
%   most consistent. Because the motion/measurement models are
%   nonlinear, the graph is relinearized and re-solved a few times
%   (Gauss-Newton), each time starting from the previous solution.
%
%   Inputs:
%       poseHistory        - Tx3 dead-reckoned poses [x,y,theta], used as
%                             the initial guess for every pose node
%       controlHistory     - (T-1)x3 odometry controls [v,w,dt] used to
%                             move from poseHistory(t,:) to poseHistory(t+1,:)
%       measurementLog      - Mx4 observations [poseIdx, landmarkId, range, bearing]
%       landmarkEstimates   - Nx2 naive initial guess for each landmark
%       observedLandmarks   - Nx1 logical, true for landmarks seen at least once
%       simParams           - simulation parameters (noise std devs)
%       Q                   - 2x2 measurement noise covariance [range; bearing]
%
%   Outputs:
%       poses     - Tx3 optimized poses [x,y,theta]
%       landmarks - Nx2 optimized landmark positions (NaN for unobserved ones)

    numPoses = size(poseHistory, 1);
    numLandmarks = size(landmarkEstimates, 1);

    % Only observed landmarks get a node in the graph; an unobserved
    % landmark has no constraints and would leave the graph singular.
    observedIds = find(observedLandmarks)';
    numObserved = numel(observedIds);
    landmarkNodeOf = zeros(numLandmarks, 1);
    landmarkNodeOf(observedIds) = 1:numObserved;

    poseDims = 3*numPoses;
    n = poseDims + 2*numObserved;

    % Pack the initial guess (dead reckoning + naive landmark triangulation)
    % into one big state vector: mu = [pose1; pose2; ...; lm1; lm2; ...]
    mu = zeros(n, 1);
    mu(1:poseDims) = reshape(poseHistory', [], 1);
    mu(poseDims+1:end) = reshape(landmarkEstimates(observedIds,:)', [], 1);

    omega_meas = inv(Q);
    numIterations = 5;
    for iter = 1:numIterations
        H = zeros(n);
        b = zeros(n, 1);

        % Anchor the first pose to its known starting value. Without this,
        % every constraint is *relative*, so the whole graph is free to
        % translate/rotate rigidly and the system is underdetermined.
        H(1:3,1:3) = H(1:3,1:3) + diag([1e6, 1e6, 1e6]);

        % ---------------- motion constraints ----------------
        for t = 1:numPoses-1
            idx_t  = 3*(t-1)+1 : 3*(t-1)+3;
            idx_t1 = 3*t+1     : 3*t+3;

            theta = mu(idx_t(3));
            v = controlHistory(t,1); w = controlHistory(t,2); dt = controlHistory(t,3);

            % error between where pose t+1 actually is and where the
            % motion model predicts it should be, given pose t and control t
            predicted = [mu(idx_t(1)) + v*cos(theta)*dt;
                         mu(idx_t(2)) + v*sin(theta)*dt;
                         wrapToPi(theta + w*dt)];
            e = mu(idx_t1) - predicted;
            e(3) = wrapToPi(e(3));

            G = [1, 0, -v*sin(theta)*dt;
                 0, 1,  v*cos(theta)*dt;
                 0, 0,  1];
            A = -G;      % d(e)/d(pose_t)
            B = eye(3);  % d(e)/d(pose_t+1)

            % Approximate odometry noise directly in pose-space as
            % independent x/y/theta noise. (The two noisy inputs, v and w,
            % only span 2 of these 3 dimensions, so mapping noise through
            % the motion model the way the EKF does would make this
            % information matrix singular.) This should reflect the
            % uncertainty in the *odometry reading* logged in
            % controlHistory, not the true motion's actuation noise.
            posVariance = (simParams.sigma_v_odom*dt)^2;
            angVariance = (simParams.sigma_w_odom*dt)^2;
            omega_motion = diag([1/posVariance, 1/posVariance, 1/angVariance]);

            H(idx_t,  idx_t)  = H(idx_t,  idx_t)  + A'*omega_motion*A;
            H(idx_t,  idx_t1) = H(idx_t,  idx_t1) + A'*omega_motion*B;
            H(idx_t1, idx_t)  = H(idx_t1, idx_t)  + B'*omega_motion*A;
            H(idx_t1, idx_t1) = H(idx_t1, idx_t1) + B'*omega_motion*B;
            b(idx_t)  = b(idx_t)  + A'*omega_motion*e;
            b(idx_t1) = b(idx_t1) + B'*omega_motion*e;
        end

        % ---------------- measurement constraints ----------------
        for k = 1:size(measurementLog, 1)
            t = measurementLog(k,1);
            j = measurementLog(k,2);
            z = measurementLog(k,3:4)';

            idx_pose = 3*(t-1)+1 : 3*(t-1)+3;
            node = landmarkNodeOf(j);
            idx_lm = poseDims + 2*(node-1)+1 : poseDims + 2*(node-1)+2;

            pose = mu(idx_pose);
            lm   = mu(idx_lm);

            delta = lm - pose(1:2);
            q = delta'*delta;
            zhat = [sqrt(q); wrapToPi(atan2(delta(2), delta(1)) - pose(3))];
            e = z - zhat;
            e(2) = wrapToPi(e(2));

            H_low = (1/q)*[-sqrt(q)*delta(1), -sqrt(q)*delta(2), 0,     sqrt(q)*delta(1), sqrt(q)*delta(2); ...
                             delta(2),         -delta(1),        -q,   -delta(2),         delta(1)];
            A = -H_low(:,1:3);  % d(e)/d(pose)
            B = -H_low(:,4:5);  % d(e)/d(landmark)

            H(idx_pose, idx_pose) = H(idx_pose, idx_pose) + A'*omega_meas*A;
            H(idx_pose, idx_lm)   = H(idx_pose, idx_lm)   + A'*omega_meas*B;
            H(idx_lm,   idx_pose) = H(idx_lm,   idx_pose) + B'*omega_meas*A;
            H(idx_lm,   idx_lm)   = H(idx_lm,   idx_lm)   + B'*omega_meas*B;
            b(idx_pose) = b(idx_pose) + A'*omega_meas*e;
            b(idx_lm)   = b(idx_lm)   + B'*omega_meas*e;
        end

        % Gauss-Newton update: solve H*dx = -b and apply the correction
        dx = -(H\b);
        mu = mu + dx;
        for t = 1:numPoses
            mu(3*t) = wrapToPi(mu(3*t));
        end
    end

    poses = reshape(mu(1:poseDims), 3, numPoses)';
    landmarks = nan(numLandmarks, 2);
    landmarks(observedIds,:) = reshape(mu(poseDims+1:end), 2, numObserved)';
end

