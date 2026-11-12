function scenario = controlTest()
%CONTROLTEST Create a deterministic CBF-QP leader-visibility scenario.
%
%   The leader starts near the observer's range cap and moves forward under
%   a constant body-frame reference. The observer has a zero nominal reference,
%   so CBF-QP corrections are the only observer motion in this scenario.

% Start from the standard runner configuration, then isolate policy intervention.
scenario = scenarios.movingPair();
scenario.Name = "CBF-QP leader visibility control test";
scenario.Time = struct('Start', 0, 'Stop', 10, 'Step', 0.05);
scenario.Observer.InitialPose = [0, 0, 0];
scenario.Observer.Reference = struct('Times', 0, 'Values', [0, 0, 0]);
scenario.Leader.InitialPose = [8, 1, 0];
scenario.Leader.Reference = struct('Times', 0, 'Values', [0.4, 0, 0]);
scenario.Obstacles = struct('Name', {}, 'Vertices', {});
% Oracle baseline predictor: re-cast the true obstacles at each queried pose.
oraclePredictor = @(belief, pose) fov.visibleBoundaryAt( ...
    pose, scenario.Observer.Fov, scenario.Obstacles, scenario.Sensor.NumRays);
scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy( ...
    struct('BoundaryPredictor', oraclePredictor));
scenario.Playback.Bounds = [-2, 14, -6, 6];
end
