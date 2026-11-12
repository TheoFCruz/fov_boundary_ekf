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

if nargin < 4
    error('fov:isPointVisible:InvalidArguments', ...
        'observerPose, fovSpec, obstacles, and point are required.');
end
if nargin < 5 || isempty(tolerance)
    tolerance = 1e-10;
end

if ~(isnumeric(observerPose) && isreal(observerPose) && ...
        numel(observerPose) == 3 && all(isfinite(observerPose(:))))
    error('fov:isPointVisible:InvalidPose', ...
        'observerPose must contain three finite real numeric values.');
end
if ~isa(fovSpec, 'fov.FovSpec')
    error('fov:isPointVisible:InvalidFov', 'fovSpec must be an fov.FovSpec object.');
end
if ~isempty(obstacles) && (~isstruct(obstacles) || ~isfield(obstacles, 'Vertices'))
    error('fov:isPointVisible:InvalidObstacles', ...
        'obstacles must be a struct array created by fov.polygonObstacle.');
end
if ~(isnumeric(point) && isreal(point) && numel(point) == 2 && ...
        all(isfinite(point(:))))
    error('fov:isPointVisible:InvalidPoint', ...
        'point must contain two finite real numeric values.');
end
if ~(isnumeric(tolerance) && isscalar(tolerance) && isreal(tolerance) && ...
        isfinite(tolerance) && tolerance > 0)
    error('fov:isPointVisible:InvalidTolerance', ...
        'tolerance must be a finite positive scalar.');
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

% Occlusion: blocked only if an edge crosses the segment before the point.
for obstacleIndex = 1:numel(obstacles)
    [edgeStarts, edgeEnds] = fov.internal.polygonEdges(obstacles(obstacleIndex));
    hitDistances = fov.internal.raySegmentIntersection(observerPose(1:2), ...
        offset, edgeStarts, edgeEnds, tolerance);
    if any(isfinite(hitDistances) & hitDistances < distance - tolerance)
        visible = false;
        return;
    end
end
visible = true;
end
