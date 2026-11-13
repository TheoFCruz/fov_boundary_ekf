function boundary = visibleBoundaryAt(pose, fovSpec, obstacles, numRays)
%VISIBLEBOUNDARYAT Oracle visible polygon seen from a world pose.
%
%   boundary = fov.visibleBoundaryAt(pose, fovSpec, obstacles, numRays)
%
% Casts numRays noiseless rays from pose = [x, y, yaw] against the true
% obstacles and returns fov.castRays' VisibleBoundary, a closed world-frame
% polygon. This is oracle geometry. Scenario factories wrap it as the oracle
% boundary predictor for the CBF policy,
%
%   predictor = @(belief, pose) fov.visibleBoundaryAt(pose, fovSpec, obstacles, numRays);
%
% which ignores the belief. It must never reach an estimator.

pose = reshape(double(pose), 1, 3);
observer = fov.Observer('Position', pose(1:2), 'Heading', pose(3), 'Fov', fovSpec);
cast = fov.castRays(observer, obstacles, 'NumRays', numRays);
boundary = cast.VisibleBoundary;
end
