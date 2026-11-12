function source = makeOracleBoundarySource(obstacles, fovSpec, numRays)
%MAKEORACLEBOUNDARYSOURCE Oracle re-cast boundary source for the CBF policy.
%
%   source = control.makeOracleBoundarySource(obstacles, fovSpec, numRays)
%
% This is explicitly oracle data: the returned handle re-casts rays against
% the true obstacle geometry at each queried pose (as in Zhou et al.), not
% belief-derived geometry. It is the oracle-boundary baseline used ahead of
% the boundary EKF. obstacles and fovSpec are captured by value from the
% scenario at construction time, so editing scenario.Obstacles afterwards
% does not change what this source returns. The handle is built outside
% simulation.runScenario from scenario obstacles; the runner itself still
% passes policies only the observation, never obstacles directly. Once the
% EKF mean transport exists, it can plug into the same BoundarySource seam
% as a non-oracle alternative.

if ~isa(fovSpec, 'fov.FovSpec')
    error('control:makeOracleBoundarySource:InvalidFov', ...
        'fovSpec must be an fov.FovSpec object.');
end
if ~isempty(obstacles) && (~isstruct(obstacles) || ~isfield(obstacles, 'Vertices'))
    error('control:makeOracleBoundarySource:InvalidObstacles', ...
        'obstacles must be a struct array created by fov.polygonObstacle.');
end
if ~(isnumeric(numRays) && isscalar(numRays) && isreal(numRays) && ...
        isfinite(numRays) && numRays >= 2 && floor(numRays) == numRays)
    error('control:makeOracleBoundarySource:InvalidNumRays', ...
        'numRays must be an integer scalar greater than or equal to 2.');
end

source = @(observation, pose) oracleBoundary(obstacles, fovSpec, numRays, pose);
end

function boundary = oracleBoundary(obstacles, fovSpec, numRays, pose)
%ORACLEBOUNDARY Re-cast against the true obstacles at the queried pose.

pose = reshape(double(pose), 1, 3);
observer = fov.Observer('Position', pose(1:2), 'Heading', pose(3), 'Fov', fovSpec);
raw = fov.castRays(observer, obstacles, 'NumRays', numRays);
boundary = raw.VisibleBoundary;
end
