function scenario = clutteredField()
%CLUTTEREDFIELD A leader explores forward through a field of small posts.
%
%   Short range: a 1 m, 100 deg FoV with 81 rays. The leader starts 0.5 m
%   ahead of the observer and walks an open, weaving path that never
%   returns, about 4.6 m over 28 s (about 0.165 m/s, a third of the 0.5 m/s
%   input bound), then stops for the last 2 s. 0.1 m posts sit on a
%   staggered grid with 0.6 m spacing, removed within 0.25 m of the leader's
%   path and 0.5 m of the observer's start, so posts cross the line of
%   sight as the pair moves. The observer holds a zero nominal reference: it
%   moves only when the CBF-QP keeps the leader in view, so it must follow
%   the leader out of range and around posts.
%
%   Measured with the oracle-boundary CBF (single run, noiseless, 2026-11-15):
%   a static observer sees the leader on 11% of the samples; the CBF keeps
%   it visible on 100%, with the constraint active on 86% of the steps and
%   never infeasible. Slower leaders (40-60 s runs) behave the same and only
%   take longer.

% Start from the standard runner configuration, then shrink the geometry.
scenario = scenarios.movingPair();
scenario.Name = "Leader explores through a cluttered field";
scenario.Time = struct('Start', 0, 'Stop', 30, 'Step', 0.025);
scenario.Observer.InitialPose = [0, 0, 0];
scenario.Observer.Fov = fov.FovSpec(1, deg2rad(100));
% A zero reference isolates the CBF-QP correction as the only observer motion.
scenario.Observer.Reference = struct('Times', 0, 'Values', [0, 0, 0]);

% Open exploring path: constant-speed legs between waypoints, then a stop.
% Leg durations are rounded to the time grid; zero yaw keeps body = world.
waypoints = [0.5, 0.0; 1.6, 0.55; 2.3, 0.15; 2.8, -0.55; 3.5, -0.2; 4.5, 0.5; 3.5, 1.0];
legLengths = vecnorm(diff(waypoints), 2, 2);
speed = sum(legLengths) / (scenario.Time.Stop - 2);
legDurations = round(legLengths / speed / scenario.Time.Step) * scenario.Time.Step;
legStarts = [0; cumsum(legDurations(1:end-1))];
legVelocities = diff(waypoints) ./ legDurations;
scenario.Leader.InitialPose = [waypoints(1, :), 0];
scenario.Leader.Reference = struct( ...
    'Times', [legStarts; sum(legDurations)], ...
    'Values', [legVelocities, zeros(numel(legStarts), 1); 0, 0, 0]);

% Staggered grid of 0.1 m posts, thinned near the path and the start.
spacing = 0.6;
pathClearance = 0.25;
startClearance = 0.5;
centers = zeros(0, 2);
rowYs = -1.2:spacing * sqrt(3) / 2:1.2;
for row = 1:numel(rowYs)
    rowOffset = mod(row, 2) * spacing / 2;
    for x = 0.6 + rowOffset:spacing:4.6
        center = [x, rowYs(row)];
        if distanceToPath(center, waypoints) >= pathClearance ...
                && norm(center) > startClearance
            centers(end + 1, :) = center; %#ok<AGROW>
        end
    end
end
square = 0.05 * [-1, -1; 1, -1; 1, 1; -1, 1];
obstacles = cell(1, size(centers, 1));
for index = 1:size(centers, 1)
    obstacles{index} = fov.polygonObstacle( ...
        sprintf('post%d', index), centers(index, :) + square);
end
scenario.Obstacles = [obstacles{:}];

% Oracle baseline predictor: re-cast the true obstacles at each queried pose.
oraclePredictor = @(belief, pose) fov.visibleBoundaryAt( ...
    pose, scenario.Observer.Fov, scenario.Obstacles, scenario.Sensor.NumRays);
scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy( ...
    struct('BoundaryPredictor', oraclePredictor));
scenario.Playback.Bounds = [-0.5, 4.8, -1.6, 1.6];
end

function distance = distanceToPath(point, waypoints)
%DISTANCETOPATH Shortest distance from a point to the polyline of waypoints.

distance = inf;
for leg = 1:size(waypoints, 1) - 1
    legStart = waypoints(leg, :);
    legVector = waypoints(leg + 1, :) - legStart;
    t = max(0, min(1, dot(point - legStart, legVector) / dot(legVector, legVector)));
    distance = min(distance, norm(point - (legStart + t * legVector)));
end
end
