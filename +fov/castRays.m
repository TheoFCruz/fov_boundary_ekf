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
%
% VisibleBoundary is an ordered, closed polygonal approximation of the
% visible region. For a partial FOV, it begins and ends at the observer;
% for a full-circle FOV, it begins and ends at the first sampled endpoint.
% Its accuracy depends on NumRays.

% Callers pass a validated fov.Observer and fov.polygonObstacle structs;
% scenario validation and the oracle source check NumRays and Tolerance.
parser = inputParser;
addParameter(parser, 'NumRays', 181);
addParameter(parser, 'Tolerance', 1e-10);
parse(parser, varargin{:});
numRays = parser.Results.NumRays;
tolerance = parser.Results.Tolerance;

rayAngles = fov.sampleRayAngles(observer, numRays);
directions = [cos(rayAngles), sin(rayAngles)];

% Distance from the observer along every ray to every obstacle edge
% (rays-by-edges, Inf on a miss).
[edgeStarts, edgeEnds, edgeObstacleId] = fov.internal.obstacleEdges(obstacles);
edgeDistances = fov.internal.rayEdgeDistances(observer.Position, directions, ...
    edgeStarts, edgeEnds, tolerance);

% Each ray stops at its nearest edge if that is strictly inside the range
% cap. Edges are stacked in obstacle order, so on a tie min picks the
% lowest obstacle index.
[nearestDistances, nearestEdges] = min(edgeDistances, [], 2);
distances = repmat(observer.MaxRange, numRays, 1);
hitObstacleId = zeros(numRays, 1);
isReturn = nearestDistances < observer.MaxRange;
distances(isReturn) = nearestDistances(isReturn);
hitObstacleId(isReturn) = edgeObstacleId(nearestEdges(isReturn));

% World endpoint of every ray.
endPoints = observer.Position + distances .* directions;

% A depth jump is a change of hit identity between consecutive rays: a
% different obstacle, or an obstacle next to a no-return. Convex obstacles
% never self-occlude, so identity alone distinguishes a true depth jump.
nextHit = [hitObstacleId(2:end); hitObstacleId(1)];
isDepthJump = hitObstacleId ~= nextHit;
if observer.OpeningAngle ~= 2*pi
    isDepthJump = isDepthJump(1:end-1);
end

% VisibleBoundary steps radially at depth jumps instead of bridging them.
visibleBoundary = fov.boundaryFromRanges([observer.Position, observer.Heading], ...
    rayAngles - observer.Heading, distances, observer.OpeningAngle, isDepthJump);

result = struct();
result.Origin = observer.Position;
result.RayAngles = rayAngles;
result.Distances = distances;
result.EndPoints = endPoints;
result.IsOccluded = hitObstacleId > 0;
result.HitObstacleId = hitObstacleId;
result.IsDepthJump = isDepthJump;
result.MaxRange = observer.MaxRange;
result.OpeningAngle = observer.OpeningAngle;
result.NumRays = numRays;
result.Tolerance = tolerance;
result.VisibleBoundary = visibleBoundary;
end
