function scenario = controlTest()
%CONTROLTEST Create a deterministic CBF-QP follower-visibility scenario.
%
%   The follower starts near the observer's range cap and moves forward under
%   a constant body-frame reference. The observer has a zero nominal reference,
%   so CBF-QP corrections are the only observer motion in this scenario.

% Start from the standard runner configuration, then isolate policy intervention.
scenario = scenarios.movingPair();
scenario.Name = "CBF-QP follower visibility control test";
scenario.Time = struct('Start', 0, 'Stop', 10, 'Step', 0.05);
scenario.Observer.InitialPose = [0, 0, 0];
scenario.Observer.Reference = struct('Times', 0, 'Values', [0, 0, 0]);
scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy();
scenario.Follower.InitialPose = [8, 1, 0];
scenario.Follower.Reference = struct('Times', 0, 'Values', [0.4, 0, 0]);
scenario.Obstacles = struct('Name', {}, 'Vertices', {});
scenario.Playback.Bounds = [-2, 14, -6, 6];
end
