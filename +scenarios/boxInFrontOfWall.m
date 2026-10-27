function scenario = boxInFrontOfWall()
%BOXINFRONTOFWALL Plan scenario S2: a follower passes behind a box occluder.
%
%   The observer holds a zero nominal reference, so the oracle-boundary CBF-QP
%   policy is the only source of observer motion. The follower walks through
%   the box's shadow against the far wall at about t = 5.4 s.

% Start from the standard runner configuration, then exercise real occlusion.
scenario = scenarios.movingPair();
scenario.Name = "Follower passes behind a box in front of a wall";
scenario.Time = struct('Start', 0, 'Stop', 16, 'Step', 0.05);
scenario.Observer.InitialPose = [0, 0, 0];
% A zero reference isolates the CBF-QP correction as the only observer motion.
scenario.Observer.Reference = struct('Times', 0, 'Values', [0, 0, 0]);
scenario.Follower.InitialPose = [7, 2.5, 0];
% This body-frame reference walks the follower through the box's shadow.
scenario.Follower.Reference = struct('Times', 0, 'Values', [0, -0.3, 0]);
scenario.Obstacles = [fov.polygonObstacle('box', ...
    [4, -0.5; 5, -0.5; 5, 0.5; 4, 0.5]), fov.polygonObstacle('wall', ...
    [9, -5; 9.3, -5; 9.3, 5; 9, 5])];
scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy(struct( ...
    'BoundarySource', control.makeOracleBoundarySource(scenario.Obstacles, ...
    scenario.Observer.Fov, scenario.Sensor.NumRays)));
scenario.Playback.Bounds = [-2, 12, -7, 7];
end
