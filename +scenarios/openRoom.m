function scenario = openRoom()
%OPENROOM Plan scenario S1: an enclosed room with no interior obstacles.
%
%   Four thin wall rectangles enclose the interior x in [-0.5, 1.7],
%   y in [-1.3, 1.3]. With the 2 m / 100 deg FoV at the origin, looking down
%   +x, the front wall returns for |theta| <~ 31.8 deg (distance to the
%   x = 1.7 face reaches the 2 m range cap there); the room corners are at
%   range 2.14 m, beyond the cap, so there are two no-return wedges for
%   31.8 deg <~ |theta| <~ 40.5 deg; the side walls return beyond that. This
%   scenario tests transport and newly-explored beams (docs/
%   matlab-implementation-plan.md sec. 9, S1) without any depth jump between
%   neighbouring covered beams. The observer rotates in place, then
%   translates forward, while staying inside the room; the leader is static
%   and the policy is the no-control reference policy.

% Start from the standard runner configuration, then enclose an open room.
scenario = scenarios.movingPair();
scenario.Name = "Open room (plan S1): transport and newly-explored beams";
scenario.Time = struct('Start', 0, 'Stop', 8, 'Step', 0.05);
scenario.Observer.InitialPose = [0, 0, 0];
scenario.Observer.Fov = fov.FovSpec(2, deg2rad(100));
% Rotate in place for 0-4 s, then translate forward for 4-8 s.
scenario.Observer.Reference = struct( ...
    'Times', [0; 4], ...
    'Values', [0, 0, 0.2; 0.1, 0, 0]);
scenario.Observer.Policy = @control.referencePolicy;
scenario.Leader.InitialPose = [1.2, 0, 0];
scenario.Leader.Reference = struct('Times', 0, 'Values', [0, 0, 0]);
% Four 0.1 m thick wall rectangles enclosing the interior.
scenario.Obstacles = [ ...
    fov.polygonObstacle('frontWall', [1.7, -1.4; 1.8, -1.4; 1.8, 1.4; 1.7, 1.4]), ...
    fov.polygonObstacle('topWall', [-0.6, 1.3; 1.8, 1.3; 1.8, 1.4; -0.6, 1.4]), ...
    fov.polygonObstacle('bottomWall', [-0.6, -1.4; 1.8, -1.4; 1.8, -1.3; -0.6, -1.3]), ...
    fov.polygonObstacle('backWall', [-0.6, -1.4; -0.5, -1.4; -0.5, 1.4; -0.6, 1.4])];
scenario.Sensor = struct('NumRays', 81, 'RangeNoiseStd', 0, 'Seed', 0);
scenario.Playback.Bounds = [-1.2, 2.4, -2, 2];
end
