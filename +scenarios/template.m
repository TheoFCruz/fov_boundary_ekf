function scenario = template()
%TEMPLATE Fully explicit scenario listing every configurable field.
%
%   scenario = scenarios.template() returns a runnable copy of the
%   box-in-front-of-wall experiment with every field spelled out. Copy this
%   file to start a new scenario factory, or call it and edit fields. The
%   policy captures obstacles, FoV, and ray count when it is built, so it is
%   constructed last; rebuild it after changing any of those fields.

scenario = struct();
scenario.Name = "Template scenario";

% Time grid (s). (Stop - Start) / Step must be an integer.
scenario.Time = struct('Start', 0, 'Stop', 16, 'Step', 0.05);

% Observer: world pose [x, y, yaw] (m, m, rad) and field of view.
scenario.Observer = struct();
scenario.Observer.InitialPose = [0, 0, 0];
% fov.FovSpec(maxRange [m], openingAngle [rad], at most 2*pi).
scenario.Observer.Fov = fov.FovSpec(10, deg2rad(100));
% Nominal body-frame input [vxBody, vyBody, omega] (m/s, m/s, rad/s), held from
% each breakpoint. Times start at Time.Start and align with Time.Step.
scenario.Observer.Reference = struct( ...
    'Times', [0], ...
    'Values', [0, 0, 0]);

% Follower (the target): same pose and reference conventions as the observer.
scenario.Follower = struct();
scenario.Follower.InitialPose = [7, 2.5, 0];
scenario.Follower.Reference = struct( ...
    'Times', [0], ...
    'Values', [0, -0.3, 0]);

% Base position (m), used only for the observer-base distance diagnostic.
scenario.Base = struct('Position', [-1, 0]);

% Static convex polygon obstacles. Use an empty struct array for open space:
% struct('Name', {}, 'Vertices', {}).
scenario.Obstacles = [ ...
    fov.polygonObstacle('box', [4, -0.5; 5, -0.5; 5, 0.5; 4, 0.5]), ...
    fov.polygonObstacle('wall', [9, -5; 9.3, -5; 9.3, 5; 9, 5])];

% Sensor: rays spread evenly over the opening angle, range noise std (m) on
% returns only, local RNG seed, and ray-intersection tolerance (m).
scenario.Sensor = struct( ...
    'NumRays', 81, ...
    'RangeNoiseStd', 0, ...
    'Seed', 0, ...
    'Tolerance', 1e-10);

% Estimator lifecycle (Initialize/Predict/Correct); pass-through until the EKF.
scenario.Estimator = estimation.makePassThroughEstimator();

% Replay only: axis bounds [xmin, xmax, ymin, ymax] (m), frame rate, speed.
scenario.Playback = struct( ...
    'Bounds', [-2, 12, -7, 7], ...
    'FrameRate', 20, ...
    'Speed', 1);

% Policy, built last. For no control use: scenario.Observer.Policy =
% @control.referencePolicy (applies Observer.Reference unchanged).
% BoundarySource = [] would use the frozen pass-through belief instead of the
% oracle re-cast.
boundarySource = control.makeOracleBoundarySource(scenario.Obstacles, ...
    scenario.Observer.Fov, scenario.Sensor.NumRays);
policyConfig = struct( ...
    'BoundarySource', boundarySource, ...
    'DistanceMargin', 0.1, ...              % m kept inside the visible set
    'CbfRate', 4, ...                       % gamma in hdot + gamma*h >= -slack
    'InputLower', [-0.5; -0.5; -1], ...     % body input bounds (contain zero)
    'InputUpper', [0.5; 0.5; 1], ...
    'InputWeights', diag([1, 1, 0.25]), ... % QP weights on u - u_nom
    'SlackPenalty', 1e4, ...                % lambda on slack^2
    'GradientBeamMultiples', [1, 2]);       % gradient steps in beam spacings
scenario.Observer.Policy = control.makeSignedDistanceCbfPolicy(policyConfig);
end
