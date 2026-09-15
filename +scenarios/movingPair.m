function scenario = movingPair()
%MOVINGPAIR Create the scripted checkpoint-one moving-agent scenario.

% This compact configuration exercises the deterministic checkpoint-one loop.
scenario.Name = "Moving pair around a wall";
scenario.Time = struct('Start', 0, 'Stop', 12, 'Step', 0.05);
% The observer policy receives its schedule as a replaceable reference source.
scenario.Observer = struct();
scenario.Observer.InitialPose = [0, 0, 0];
scenario.Observer.Fov = fov.FovSpec(10, deg2rad(100));
scenario.Observer.Reference = struct( ...
    'Times', [0; 4; 8], ...
    'Values', [0.15, 0, 0; 0, 0, 0.15; 0.10, 0, 0]);
scenario.Observer.Policy = @control.referencePolicy;
% The follower follows an independent scripted body-frame reference.
scenario.Follower = struct();
scenario.Follower.InitialPose = [2, -1, 0];
scenario.Follower.Reference = struct( ...
    'Times', [0; 6], 'Values', [0, 0.2, 0; 0, -0.2, 0]);
scenario.Base = struct('Position', [-1, 0]);
scenario.Obstacles = fov.polygonObstacle('wall', ...
    [4, -2; 4, 2; 4.3, 2; 4.3, -2]);
% Deterministic noiseless sensing isolates runner and estimator contracts.
scenario.Sensor = struct('NumRays', 81, 'RangeNoiseStd', 0, 'Seed', 0);
scenario.Estimator = estimation.makePassThroughEstimator();
scenario.Playback = struct( ...
    'Bounds', [-2, 12, -7, 7], 'FrameRate', 20, 'Speed', 1);
end
