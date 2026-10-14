% GENERATE_CODE_WALKTHROUGH_FIGURES  Run the real GraphSLAM simulation and export what the
% walkthrough document (graphslam_code_walkthrough.md) needs.
%
% Run (from anywhere):
%   matlab -batch "run('Graph_SLAM/Documents/images/generate_code_walkthrough_figures.m')"
% Optional: choose the random seed with the environment variable GRAPHSLAM_SEED (default 4).
% Set GRAPHSLAM_EXPORT=0 to only print the one-line SUMMARY of a run and write no files.
%
% It does not modify SIMULATION_GraphSLAM.m. It runs that script unchanged, and then
%   1. saves the final figure that the simulation itself draws         -> code_result_matlab.png
%   2. saves the logged data and per-iteration solver diagnostics       -> code_walkthrough_run.json
% The diagnostics come from an instrumented copy of the solver's system-building loops (the local
% function buildSystem below). The script checks that the copy reproduces the simulation's own
% optimized poses and landmarks, so the diagnostics describe the real result.
% Python then draws the remaining figures from the JSON file:
%   python Graph_SLAM/Documents/images/generate_code_walkthrough_figures.py

seed = str2double(getenv('GRAPHSLAM_SEED'));
if isnan(seed), seed = 4; end
setappdata(0, 'walkthroughSeed', seed);   % `clear all` inside the simulation wipes the workspace
setappdata(0, 'walkthroughExport', ~strcmp(getenv('GRAPHSLAM_EXPORT'), '0'));

here = fileparts(mfilename('fullpath'));
cd(here); cd(fullfile('..', '..'));
setappdata(0, 'walkthroughDir', pwd);

rng(seed);                                % clear all does not reset the random number generator
run(fullfile(getappdata(0, 'walkthroughDir'), 'SIMULATION_GraphSLAM.m'));

% ----- everything above was cleared by the simulation; recover what we need -----
seed = getappdata(0, 'walkthroughSeed');
doExport = getappdata(0, 'walkthroughExport');
here = fullfile(getappdata(0, 'walkthroughDir'), 'Documents', 'images');
if doExport
    exportgraphics(gcf, fullfile(here, 'code_result_matlab.png'), 'Resolution', 150);
end

% ----- instrumented copy of the solver's system building -----
numPoses = size(poseHistory, 1);
numLandmarks = size(landmarkEstimates, 1);
observedIds = find(observedLandmarks)';
numObserved = numel(observedIds);
landmarkNodeOf = zeros(numLandmarks, 1);
landmarkNodeOf(observedIds) = 1:numObserved;
poseDims = 3*numPoses;
n = poseDims + 2*numObserved;

mu = zeros(n, 1);
mu(1:poseDims) = reshape(poseHistory', [], 1);
mu(poseDims+1:end) = reshape(landmarkEstimates(observedIds,:)', [], 1);

omega_meas = inv(Q);
numIterations = 5;
costBefore = zeros(numIterations, 1);
maxStep = zeros(numIterations, 1);
for iter = 1:numIterations
    [H, b, cost, costMotion] = buildSystem(mu, controlHistory, measurementLog, landmarkNodeOf, ...
                               numPoses, poseDims, n, omega_meas, R);
    if iter == 1
        costMotionFirst = costMotion;
        [hr, hc] = find(H);
        patternRows = hr; patternCols = hc;
        nnzH = nnz(H);
    end
    costBefore(iter) = cost;
    if iter == 1
        % Cholesky factor size for the first linearization, in the state-vector order and in the
        % order chosen by MATLAB's approximate-minimum-degree routine (symamd).
        Hs = sparse((H + H')/2);
        nnzCholNatural = nnz(chol(Hs, 'lower'));
        perm = symamd(Hs);
        nnzCholSymamd = nnz(chol(Hs(perm, perm), 'lower'));
        landmarksFirst = [poseDims+1:n, 1:poseDims];
        nnzCholLandmarksFirst = nnz(chol(Hs(landmarksFirst, landmarksFirst), 'lower'));
    end
    dx = -(H\b);
    maxStep(iter) = max(abs(dx));
    mu = mu + dx;
    for t = 1:numPoses
        mu(3*t) = wrapToPiLocal(mu(3*t));
    end
end
[~, ~, costAfter] = buildSystem(mu, controlHistory, measurementLog, landmarkNodeOf, ...
                                numPoses, poseDims, n, omega_meas, R);

copyPoses = reshape(mu(1:poseDims), 3, numPoses)';
copyLandmarks = reshape(mu(poseDims+1:end), 2, numObserved)';
refLandmarks = optimizedLandmarks(observedIds, :);
assert(max(abs(copyPoses(:) - optimizedPoses(:))) < 1e-8, 'instrumented copy does not match graphSLAM poses');
assert(max(abs(copyLandmarks(:) - refLandmarks(:))) < 1e-8, ...
       'instrumented copy does not match graphSLAM landmarks');

% ----- one-line summary, so that any seed can be checked without exporting anything -----
posErrDeadReckoned = sqrt(sum((poseHistory(:,1:2) - truePoseHistory(:,1:2)).^2, 2));
posErrOptimized    = sqrt(sum((optimizedPoses(:,1:2) - truePoseHistory(:,1:2)).^2, 2));
fprintf('SUMMARY seed=%d poses=%d meanPosErr deadReckoned=%.4f optimized=%.4f\n', ...
        seed, numPoses, mean(posErrDeadReckoned), mean(posErrOptimized));
if ~doExport
    return
end

% ----- export -----
omegaMotionMatrix = inv(R*controlHistory(1,3)^2);
data = struct();
data.seed = seed;
data.numPoses = numPoses;
data.numMeasurements = size(measurementLog, 1);
data.n = n;
data.poseDims = poseDims;
data.numIterations = numIterations;
data.landmarks = landmarks;
data.goalPoints = goalPoints;
data.observedIds = observedIds;
data.landmarkEstimates = landmarkEstimates(observedIds, :);
data.optimizedLandmarks = optimizedLandmarks(observedIds, :);
data.truePoses = truePoseHistory;
data.deadReckoned = poseHistory;
data.optimized = optimizedPoses;
data.measurementLog = measurementLog;
data.sigma_v_odom = simParams.sigma_v_odom;
data.sigma_w_odom = simParams.sigma_w_odom;
data.sigma_r = simParams.sigma_r;
data.sigma_b = simParams.sigma_b;
data.dt = simParams.dt_s;
data.omegaMotion = diag(omegaMotionMatrix)';
data.omegaMeas = diag(omega_meas)';
data.costBefore = costBefore;
data.costMotionFirst = costMotionFirst;
data.costAfter = costAfter;
data.maxStep = maxStep;
data.nnzH = nnzH;
data.nnzCholNatural = nnzCholNatural;
data.nnzCholSymamd = nnzCholSymamd;
data.nnzCholLandmarksFirst = nnzCholLandmarksFirst;
data.patternRows = patternRows;
data.patternCols = patternCols;
fid = fopen(fullfile(here, 'code_walkthrough_run.json'), 'w');
fwrite(fid, jsonencode(data, 'PrettyPrint', false));
fclose(fid);
fprintf('Walkthrough data written for seed %d (%d poses, %d unknowns).\n', seed, numPoses, n);

% =====================================================================================
function [H, b, cost, costMotion] = buildSystem(mu, controlHistory, measurementLog, landmarkNodeOf, ...
                                    numPoses, poseDims, n, omega_meas, R)
% The loops below are copied from graphSLAM in SIMULATION_GraphSLAM.m. The only addition is the
% running `cost`, the sum of e'*omega*e over every constraint, which the original does not compute.
% `costMotion` is the part of it that comes from the motion constraints.
    H = zeros(n);
    b = zeros(n, 1);
    cost = 0;
    costMotion = 0;

    H(1:3,1:3) = H(1:3,1:3) + diag([1e6, 1e6, 1e6]);

    for t = 1:numPoses-1
        idx_t  = 3*(t-1)+1 : 3*(t-1)+3;
        idx_t1 = 3*t+1     : 3*t+3;

        theta = mu(idx_t(3));
        v = controlHistory(t,1); w = controlHistory(t,2); dt = controlHistory(t,3);

        predicted = [mu(idx_t(1)) + v*cos(theta)*dt;
                     mu(idx_t(2)) + v*sin(theta)*dt;
                     wrapToPiLocal(theta + w*dt)];
        e = mu(idx_t1) - predicted;
        e(3) = wrapToPiLocal(e(3));

        G = [1, 0, -v*sin(theta)*dt;
             0, 1,  v*cos(theta)*dt;
             0, 0,  1];
        A = -G;
        B = eye(3);

        omega_motion = inv(R*dt^2);

        H(idx_t,  idx_t)  = H(idx_t,  idx_t)  + A'*omega_motion*A;
        H(idx_t,  idx_t1) = H(idx_t,  idx_t1) + A'*omega_motion*B;
        H(idx_t1, idx_t)  = H(idx_t1, idx_t)  + B'*omega_motion*A;
        H(idx_t1, idx_t1) = H(idx_t1, idx_t1) + B'*omega_motion*B;
        b(idx_t)  = b(idx_t)  + A'*omega_motion*e;
        b(idx_t1) = b(idx_t1) + B'*omega_motion*e;
        cost = cost + e'*omega_motion*e;
        costMotion = costMotion + e'*omega_motion*e;
    end

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
        zhat = [sqrt(q); wrapToPiLocal(atan2(delta(2), delta(1)) - pose(3))];
        e = z - zhat;
        e(2) = wrapToPiLocal(e(2));

        H_low = (1/q)*[-sqrt(q)*delta(1), -sqrt(q)*delta(2), 0,     sqrt(q)*delta(1), sqrt(q)*delta(2); ...
                         delta(2),         -delta(1),        -q,   -delta(2),         delta(1)];
        A = -H_low(:,1:3);
        B = -H_low(:,4:5);

        H(idx_pose, idx_pose) = H(idx_pose, idx_pose) + A'*omega_meas*A;
        H(idx_pose, idx_lm)   = H(idx_pose, idx_lm)   + A'*omega_meas*B;
        H(idx_lm,   idx_pose) = H(idx_lm,   idx_pose) + B'*omega_meas*A;
        H(idx_lm,   idx_lm)   = H(idx_lm,   idx_lm)   + B'*omega_meas*B;
        b(idx_pose) = b(idx_pose) + A'*omega_meas*e;
        b(idx_lm)   = b(idx_lm)   + B'*omega_meas*e;
        cost = cost + e'*omega_meas*e;
    end
end

function wrapped_rad = wrapToPiLocal(angle_rad)
    wrapped_rad = mod(angle_rad + pi, 2*pi) - pi;
end
