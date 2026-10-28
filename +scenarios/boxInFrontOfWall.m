function scenario = boxInFrontOfWall()
%BOXINFRONTOFWALL Plan scenario S2: a follower passes behind a box occluder.
%
%   A compact layout: a 0.2 m box in front of a short wall, viewed through a
%   2 m, 100 deg FoV with 81 rays. The short range keeps ray endpoints about
%   3 cm apart at the target without more rays. The observer holds a zero
%   nominal reference, so the oracle-boundary CBF-QP policy is the only source
%   of observer motion. The follower walks at 0.1 m/s (a fifth of the 0.5 m/s
%   input bound) through the box's shadow between about t = 4 s and t = 8 s.

% Start from the standard runner configuration, then exercise real occlusion.
scenario = scenarios.movingPair();
scenario.Name = "Follower passes behind a box in front of a wall";
scenario.Time = struct('Start', 0, 'Stop', 12, 'Step', 0.05);
scenario.Observer.InitialPose = [0, 0, 0];
scenario.Observer.Fov = fov.FovSpec(2, deg2rad(100));
% A zero reference isolates the CBF-QP correction as the only observer motion.
scenario.Observer.Reference = struct('Times', 0, 'Values', [0, 0, 0]);
scenario.Follower.InitialPose = [1.4, 0.6, 0];
% This body-frame reference walks the follower through the box's shadow.
scenario.Follower.Reference = struct('Times', 0, 'Values', [0, -0.1, 0]);
scenario.Obstacles = [fov.polygonObstacle('box', ...
    [0.8, -0.1; 1, -0.1; 1, 0.1; 0.8, 0.1]), fov.polygonObstacle('wall', ...
    [1.8, -1; 1.86, -1; 1.86, 1; 1.8, 1])];
scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy(struct( ...
    'BoundarySource', control.makeOracleBoundarySource(scenario.Obstacles, ...
    scenario.Observer.Fov, scenario.Sensor.NumRays)));
scenario.Playback.Bounds = [-0.5, 2.5, -1.5, 1.5];
end
