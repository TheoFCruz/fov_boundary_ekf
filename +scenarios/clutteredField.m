function scenario = clutteredField()
%CLUTTEREDFIELD A slow follower weaves through small scattered obstacles.
%
%   Loosely modeled on the cluttered environment of the VisCoNetwork paper,
%   at the compact scale of scenarios.boxInFrontOfWall (2 m, 100 deg FoV, 81
%   rays). The follower starts about 1.5 m out and visits four waypoints at
%   about 0.1 m/s (a fifth of the 0.5 m/s input bound), keeping at least
%   0.2 m from every post. The first leg reaches the 2 m range limit within
%   a few seconds and a later leg nears the FoV edge, so the observer must
%   translate and turn a little; otherwise it holds still. No 0.1 m post lies
%   on the line of sight: crossing behind a post at range fraction s would
%   force the observer to slide about s/(1-s) times the follower's lateral
%   motion. The posts sit beside or behind the path instead. The observer
%   holds a zero reference and runs the Zhou-style oracle-boundary CBF-QP.

% Start from the standard runner configuration, then shrink the geometry.
scenario = scenarios.movingPair();
scenario.Name = "Follower weaves through a cluttered field";
scenario.Time = struct('Start', 0, 'Stop', 32, 'Step', 0.05);
scenario.Observer.InitialPose = [0, 0, 0];
scenario.Observer.Fov = fov.FovSpec(2, deg2rad(100));
% A zero reference isolates the CBF-QP correction as the only observer motion.
scenario.Observer.Reference = struct('Times', 0, 'Values', [0, 0, 0]);

% Constant-velocity legs between waypoints; zero yaw keeps body = world frame.
waypoints = [1.45, 0.3; 2.05, 0.3; 1.3, -0.35; 0.75, -0.95; 1.2, 0.15];
legStarts = [0; 5; 13; 21];
legDurations = diff([legStarts; scenario.Time.Stop]);
legVelocities = diff(waypoints) ./ legDurations;
scenario.Follower.InitialPose = [waypoints(1, :), 0];
scenario.Follower.Reference = struct('Times', legStarts, ...
    'Values', [legVelocities, zeros(numel(legStarts), 1)]);

% Side and background clutter, at least 0.2 m from the follower's path.
centers = [0.6, 0.45; 1.85, -0.25; 0.3, -0.6; 1.2, 0.75; 1.25, -0.85; ...
    1.75, 0.7; 0.25, 0.6];
halfWidth = 0.05;
square = halfWidth * [-1, -1; 1, -1; 1, 1; -1, 1];
obstacles = cell(1, size(centers, 1));
for index = 1:size(centers, 1)
    obstacles{index} = fov.polygonObstacle(sprintf('post%d', index), ...
        centers(index, :) + square);
end
scenario.Obstacles = [obstacles{:}];

scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy(struct( ...
    'BoundarySource', control.makeOracleBoundarySource(scenario.Obstacles, ...
    scenario.Observer.Fov, scenario.Sensor.NumRays)));
scenario.Playback.Bounds = [-0.5, 2.5, -1.5, 1.5];
end
