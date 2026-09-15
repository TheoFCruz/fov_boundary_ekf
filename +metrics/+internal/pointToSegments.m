function [distance, closestPoints, closestEdgeIndex] = pointToSegments( ...
        queryPoints, edgeStarts, edgeEnds, varargin)
%POINTTOSEGMENTS Find the closest point on a set of segments for each point.

parser = inputParser;
parser.FunctionName = 'metrics.internal.pointToSegments';
addParameter(parser, 'ChunkSize', 8192);
parse(parser, varargin{:});
chunkSize = parser.Results.ChunkSize;

if ~(isnumeric(queryPoints) && isreal(queryPoints) && ismatrix(queryPoints) && ...
        size(queryPoints, 2) == 2 && all(isfinite(queryPoints(:))))
    error('metrics:internal:pointToSegments:InvalidQueryPoints', ...
        'queryPoints must be a finite real N-by-2 numeric array.');
end

if ~(isnumeric(edgeStarts) && isnumeric(edgeEnds) && isreal(edgeStarts) && ...
        isreal(edgeEnds) && isequal(size(edgeStarts), size(edgeEnds)) && ...
        size(edgeStarts, 2) == 2 && ~isempty(edgeStarts) && ...
        all(isfinite(edgeStarts(:))) && all(isfinite(edgeEnds(:))))
    error('metrics:internal:pointToSegments:InvalidSegments', ...
        'edgeStarts and edgeEnds must be matching finite M-by-2 arrays.');
end

if ~(isnumeric(chunkSize) && isscalar(chunkSize) && isreal(chunkSize) && ...
        isfinite(chunkSize) && chunkSize >= 1 && floor(chunkSize) == chunkSize)
    error('metrics:internal:pointToSegments:InvalidChunkSize', ...
        'ChunkSize must be a positive integer scalar.');
end

% Process point blocks to avoid a large point-by-edge temporary array.
queryPoints = double(queryPoints);
edgeStarts = double(edgeStarts);
edgeEnds = double(edgeEnds);
chunkSize = double(chunkSize);
numPoints = size(queryPoints, 1);
numEdges = size(edgeStarts, 1);
edgeDirections = edgeEnds - edgeStarts;
edgeLengthSquared = sum(edgeDirections.^2, 2).';

if any(edgeLengthSquared == 0)
    error('metrics:internal:pointToSegments:DegenerateSegment', ...
        'Segments must have nonzero length.');
end

distance = zeros(numPoints, 1);
if nargout > 1
    closestPoints = zeros(numPoints, 2);
    closestEdgeIndex = zeros(numPoints, 1);
end

% Each block projects every query point onto every polygon edge.
for startIndex = 1:chunkSize:numPoints
    endIndex = min(startIndex + chunkSize - 1, numPoints);
    pointIndices = startIndex:endIndex;
    points = queryPoints(pointIndices, :);
    numChunkPoints = size(points, 1);

    relativeX = bsxfun(@minus, points(:, 1), edgeStarts(:, 1).');
    relativeY = bsxfun(@minus, points(:, 2), edgeStarts(:, 2).');
    projection = bsxfun(@times, relativeX, edgeDirections(:, 1).') + ...
        bsxfun(@times, relativeY, edgeDirections(:, 2).');
    parameter = bsxfun(@rdivide, projection, edgeLengthSquared);
    parameter = max(0, min(1, parameter));

    closestX = bsxfun(@plus, edgeStarts(:, 1).', ...
        bsxfun(@times, parameter, edgeDirections(:, 1).'));
    closestY = bsxfun(@plus, edgeStarts(:, 2).', ...
        bsxfun(@times, parameter, edgeDirections(:, 2).'));
    distanceSquared = bsxfun(@minus, points(:, 1), closestX).^2 + ...
        bsxfun(@minus, points(:, 2), closestY).^2;
    [minimumDistanceSquared, minimumEdgeIndex] = min(distanceSquared, [], 2);
    distance(pointIndices) = sqrt(minimumDistanceSquared);

    if nargout > 1
        linearIndices = sub2ind([numChunkPoints, numEdges], ...
            (1:numChunkPoints).', minimumEdgeIndex);
        closestPoints(pointIndices, :) = [closestX(linearIndices), ...
            closestY(linearIndices)];
        closestEdgeIndex(pointIndices) = minimumEdgeIndex;
    end
end
end
