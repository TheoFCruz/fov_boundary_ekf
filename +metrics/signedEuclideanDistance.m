function [distance, details] = signedEuclideanDistance(boundary, queryPoints, varargin)
%SIGNEDEUCLIDEANDISTANCE Signed Euclidean distance to a simple polygon.
%
%   distance = metrics.signedEuclideanDistance(boundary, queryPoints)
%   [distance, details] = metrics.signedEuclideanDistance(..., ...
%       'Tolerance', tolerance)
%
% boundary is an open or closed M-by-2 simple polygon. queryPoints is an
% N-by-2 array. Distance is negative strictly inside the polygon, positive
% outside it, and zero within Tolerance of its boundary.

if nargin < 2
    error('metrics:signedEuclideanDistance:InvalidArguments', ...
        'boundary and queryPoints are required.');
end

parser = inputParser;
parser.FunctionName = 'metrics.signedEuclideanDistance';
addParameter(parser, 'Tolerance', []);
parse(parser, varargin{:});
tolerance = parser.Results.Tolerance;

if ~(isnumeric(boundary) && isreal(boundary) && ismatrix(boundary) && ...
        size(boundary, 2) == 2 && all(isfinite(boundary(:))))
    error('metrics:signedEuclideanDistance:InvalidBoundary', ...
        'boundary must be a finite real M-by-2 numeric array.');
end

if ~(isnumeric(queryPoints) && isreal(queryPoints) && ismatrix(queryPoints) && ...
        size(queryPoints, 2) == 2 && all(isfinite(queryPoints(:))))
    error('metrics:signedEuclideanDistance:InvalidQueryPoints', ...
        'queryPoints must be a finite real N-by-2 numeric array.');
end

boundary = double(boundary);
queryPoints = double(queryPoints);

if isempty(tolerance)
    coordinateScale = max(1, max(abs(boundary(:))));
    tolerance = 32 * eps(coordinateScale);
elseif ~(isnumeric(tolerance) && isscalar(tolerance) && isreal(tolerance) && ...
        isfinite(tolerance) && tolerance > 0)
    error('metrics:signedEuclideanDistance:InvalidTolerance', ...
        'Tolerance must be a finite positive scalar.');
else
    tolerance = double(tolerance);
end

[boundary, edgeStarts, edgeEnds] = ...
    metrics.internal.normalizeBoundary(boundary, tolerance);

if nargout > 1
    [unsignedDistance, closestPoint, closestEdgeIndex] = ...
        metrics.internal.pointToSegments(queryPoints, edgeStarts, edgeEnds);
else
    unsignedDistance = metrics.internal.pointToSegments( ...
        queryPoints, edgeStarts, edgeEnds);
end

[inside, onBoundary] = inpolygon(queryPoints(:, 1), queryPoints(:, 2), ...
    boundary(:, 1), boundary(:, 2));
isOnBoundary = onBoundary | unsignedDistance <= tolerance;
isInside = inside & ~isOnBoundary;

unsignedDistance(isOnBoundary) = 0;
distance = unsignedDistance;
distance(isInside) = -unsignedDistance(isInside);

if nargout > 1
    details = struct();
    details.UnsignedDistance = unsignedDistance;
    details.IsInside = isInside;
    details.IsOnBoundary = isOnBoundary;
    details.ClosestPoint = closestPoint;
    details.ClosestEdgeIndex = closestEdgeIndex;
end
end
