function [u, z, truePose, correspondence] = getControlAndMeasurement(estimatedPose, v_mps, lastGoal, currentGoal, landmarks)
%GETCONTROLANDMEASUREMENT Simulates one iteration of robot motion and sensing
%   Computes a steering command via pure pursuit, advances the robot's true
%   pose with a noisy differential-drive motion model, and generates a
%   noisy range-bearing measurement to one randomly selected landmark.
%
%   Inputs:
%       estimatedPose - filter's current pose estimate [x,y,theta]; used to
%                       command the robot and to seed the true pose on the
%                       first call
%       v_mps         - commanded linear velocity (m/s)
%       lastGoal      - [x,y] start point of the current path segment
%       currentGoal   - [x,y] end point of the current path segment
%       landmarks     - Nx2 matrix of landmark [x,y] positions
%
%   Outputs:
%       u             - noisy odometry reading [v_mps, w_radps, dt_s]
%       z             - [range_m, bearing_rad] to the selected landmark
%       truePose      - robot's true pose [x,y,theta] after this iteration's motion
%       correspondence - row index into landmarks of the observed landmark

    persistent trueRobotPose;

    % Robot/sensor/noise parameters
    model = 'icr';                % kinematic model: 'icr' or 'simple'
    dt_s = 0.1;                   % time between iterations (s)
    sigma_v = 0.02;               % linear velocity noise std dev (m/s)
    sigma_w = deg2rad(1);         % angular velocity noise std dev (rad/s)
    sigma_r = 0.05;               % range measurement noise std dev (m)
    sigma_b = deg2rad(2);         % bearing measurement noise std dev (rad)

    % initialize trueRobotPose if first call
    if(isempty(trueRobotPose))
        trueRobotPose = estimatedPose;
    end

    % Pure pursuit steering command toward the current path segment
    w_radps = purePursuit(estimatedPose, lastGoal, currentGoal, v_mps);

    % Corrupt the commanded velocities with odometry noise
    v_noisy = v_mps + sigma_v*randn;
    w_noisy = w_radps + sigma_w*randn;
    u = [v_noisy, w_noisy, dt_s];

    % Advance the true pose using the noisy control
    trueRobotPose = differentialDriveKinematics(trueRobotPose, v_noisy, w_noisy, dt_s, model);

    % Generate a range/bearing measurement to one randomly selected landmark
    correspondence = randi(size(landmarks, 1));
    dx = landmarks(correspondence,1) - trueRobotPose(1);
    dy = landmarks(correspondence,2) - trueRobotPose(2);
    range_m = sqrt(dx^2 + dy^2);
    bearing_rad = wrapToPi(atan2(dy, dx) - trueRobotPose(3));
    z = [range_m + sigma_r*randn, bearing_rad + sigma_b*randn];

    truePose = trueRobotPose;
end

function [wrapped_rad] = wrapToPi(angle_rad)
    % wrap an angle in radians to the range [-pi, pi]
    wrapped_rad = mod(angle_rad + pi, 2*pi) - pi;
end
