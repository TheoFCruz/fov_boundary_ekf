function result = castRays(observer, obstacles, varargin)
%CASTRAYS Cast uniformly sampled FOV rays against polygonal obstacles.
%
%   result = fov.castRays(observer, obstacles)
%   result = fov.castRays(observer, obstacles, ...
%       'NumRays', 181, 'Tolerance', 1e-10)
%
% Each ray is clipped at the nearest obstacle intersection or at the
% observer's maximum range. HitObstacleId is zero when the ray reaches the
% maximum range without hitting an obstacle.

if nargin < 2
    error('fov:castRays:InvalidArguments', ...
        'observer and obstacles are required.');
end

if ~isa(observer, 'fov.Observer')
    error('fov:castRays:InvalidObserver', ...
        'observer must be an fov.Observer object.');
end

parser = inputParser;
parser.FunctionName = 'fov.castRays';
addParameter(parser, 'NumRays', 181);
addParameter(parser, 'Tolerance', 1e-10);
parse(parser, varargin{:});

numRays = parser.Results.NumRays;
tolerance = parser.Results.Tolerance;

if ~(isnumeric(numRays) && isscalar(numRays) && isreal(numRays) && ...
        isfinite(numRays) && numRays >= 2 && floor(numRays) == numRays)
    error('fov:castRays:InvalidNumRays', ...
        'NumRays must be an integer scalar greater than or equal to 2.');
end

if ~(isnumeric(tolerance) && isscalar(tolerance) && isreal(tolerance) && ...
        isfinite(tolerance) && tolerance > 0)
    error('fov:castRays:InvalidTolerance', ...
        'Tolerance must be a finite positive scalar.');
end

if ~isempty(obstacles) && ...
        (~isstruct(obstacles) || ~isfield(obstacles, 'Vertices'))
    error('fov:castRays:InvalidObstacles', ...
        'obstacles must be a struct array created by fov.polygonObstacle.');
end

numRays = double(numRays);
tolerance = double(tolerance);
rayAngles = fov.sampleRayAngles(observer, numRays);
directions = [cos(rayAngles), sin(rayAngles)];
numObstacles = numel(obstacles);

distances = repmat(observer.MaxRange, numRays, 1);
hitObstacleId = zeros(numRays, 1);

for rayIndex = 1:numRays
    rayOrigin = observer.Position;
    rayDirection = directions(rayIndex, :);

    for obstacleIndex = 1:numObstacles
        [edgeStarts, edgeEnds] = ...
            fov.internal.polygonEdges(obstacles(obstacleIndex));
        edgeDistances = fov.internal.raySegmentIntersection( ...
            rayOrigin, rayDirection, edgeStarts, edgeEnds, tolerance);

        validHits = isfinite(edgeDistances) & ...
            edgeDistances <= observer.MaxRange + tolerance;
        if ~any(validHits)
            continue;
        end

        obstacleDistance = min(edgeDistances(validHits));
        if obstacleDistance < distances(rayIndex)
            distances(rayIndex) = max(obstacleDistance, 0);
            hitObstacleId(rayIndex) = obstacleIndex;
        end
    end
end

endPoints = bsxfun(@plus, observer.Position, ...
    bsxfun(@times, distances, directions));
isOccluded = hitObstacleId > 0;

nominalEndPoints = bsxfun(@plus, observer.Position, ...
    observer.MaxRange * directions);

if observer.OpeningAngle == 2*pi
    nominalBoundary = [nominalEndPoints; nominalEndPoints(1, :)];
    visibleBoundary = [endPoints; endPoints(1, :)];
else
    nominalBoundary = [observer.Position; nominalEndPoints; observer.Position];
    visibleBoundary = [observer.Position; endPoints; observer.Position];
end

result = struct();
result.Origin = observer.Position;
result.RayAngles = rayAngles;
result.Directions = directions;
result.Distances = distances;
result.EndPoints = endPoints;
result.IsOccluded = isOccluded;
result.HitObstacleId = hitObstacleId;
result.MaxRange = observer.MaxRange;
result.OpeningAngle = observer.OpeningAngle;
result.NumRays = numRays;
result.Tolerance = tolerance;
result.NominalBoundary = nominalBoundary;
result.VisibleBoundary = visibleBoundary;
end
