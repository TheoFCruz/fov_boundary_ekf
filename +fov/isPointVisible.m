function visible = isPointVisible(observerPose, fovSpec, obstacles, point, tolerance)
%ISPOINTVISIBLE True line-of-sight visibility oracle for scoring only.
%
%   visible = fov.isPointVisible(observerPose, fovSpec, obstacles, point)
%   visible = fov.isPointVisible(..., tolerance)
%
% A point is visible iff it is within MaxRange, within the aperture (unless
% OpeningAngle is 2*pi), and no obstacle edge crosses the observer-to-point
% segment strictly before the point. This is oracle geometry for evaluation
% and metrics, not a filter or controller input; do not route it into the
% estimator or policy seams.

if nargin < 5
    tolerance = 1e-10;
end

observerPose = reshape(double(observerPose), 1, 3);
point = reshape(double(point), 1, 2);
offset = point - observerPose(1:2);
distance = hypot(offset(1), offset(2));

% Range-cap and aperture are checked first; a point at the observer is visible.
if distance > fovSpec.MaxRange + tolerance
    visible = false;
    return;
end
if fovSpec.OpeningAngle ~= 2*pi
    bearing = atan2(offset(2), offset(1)) - observerPose(3);
    bearing = mod(bearing + pi, 2*pi) - pi;
    if abs(bearing) > fovSpec.OpeningAngle / 2 + tolerance
        visible = false;
        return;
    end
end
if distance <= tolerance
    visible = true;
    return;
end

% Occlusion: blocked only if an edge crosses the sight line before the point.
[edgeStarts, edgeEnds] = fov.internal.obstacleEdges(obstacles);
hitDistances = fov.internal.rayEdgeDistances(observerPose(1:2), offset, ...
    edgeStarts, edgeEnds, tolerance);
visible = ~any(hitDistances < distance - tolerance);
end
