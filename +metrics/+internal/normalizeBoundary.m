function [boundary, edgeStarts, edgeEnds] = normalizeBoundary(boundary, tolerance)
%NORMALIZEBOUNDARY Validate and close a simple polygon boundary.

if ~(isnumeric(boundary) ...
        && isreal(boundary) ...
        && ismatrix(boundary) ...
        && size(boundary, 2) == 2 ...
        && all(isfinite(boundary(:))))
    error('metrics:internal:normalizeBoundary:InvalidBoundary', ...
        'boundary must be a finite real M-by-2 numeric array.');
end

if ~(isnumeric(tolerance) ...
        && isscalar(tolerance) ...
        && isreal(tolerance) ...
        && isfinite(tolerance) ...
        && tolerance > 0)
    error('metrics:internal:normalizeBoundary:InvalidTolerance', ...
        'tolerance must be a finite positive scalar.');
end

vertices = double(boundary);
tolerance = double(tolerance);

if size(vertices, 1) < 3
    error('metrics:internal:normalizeBoundary:DegenerateBoundary', ...
        'boundary must contain at least three vertices.');
end

% Keep one copy of each vertex while retaining the implicit closing edge.
if norm(vertices(1, :) - vertices(end, :)) <= tolerance
    vertices(end, :) = [];
end

if isempty(vertices)
    error('metrics:internal:normalizeBoundary:DegenerateBoundary', ...
        'boundary must contain at least three unique vertices.');
end

keep = [true; sqrt(sum(diff(vertices, 1, 1).^2, 2)) > tolerance];
vertices = vertices(keep, :);

if size(vertices, 1) > 1 && norm(vertices(1, :) - vertices(end, :)) <= tolerance
    vertices(end, :) = [];
end

if size(vertices, 1) < 3 || size(unique(vertices, 'rows'), 1) < 3
    error('metrics:internal:normalizeBoundary:DegenerateBoundary', ...
        'boundary must contain at least three unique vertices.');
end

% Reject collapsed edges before signed area and intersection diagnostics.
boundary = [vertices; vertices(1, :)];
edgeStarts = boundary(1:end-1, :);
edgeEnds = boundary(2:end, :);
edgeLengths = sqrt(sum((edgeEnds - edgeStarts).^2, 2));

if any(edgeLengths <= tolerance)
    error('metrics:internal:normalizeBoundary:DegenerateEdge', ...
        'boundary contains an edge shorter than tolerance.');
end

% A nonzero signed area distinguishes a polygon from a degenerate chain.
signedAreaTwice = sum( ...
    edgeStarts(:, 1) .* edgeEnds(:, 2) ...
    - edgeStarts(:, 2) .* edgeEnds(:, 1));
coordinateScale = max(1, max(abs(boundary(:))));
if abs(signedAreaTwice) <= 2 * tolerance * coordinateScale
    error('metrics:internal:normalizeBoundary:DegenerateBoundary', ...
        'boundary must enclose nonzero area.');
end

if hasSelfIntersection(edgeStarts, edgeEnds, tolerance)
    error('metrics:internal:normalizeBoundary:SelfIntersectingBoundary', ...
        'boundary must be a simple polygon without self-intersections.');
end
end

function intersects = hasSelfIntersection(edgeStarts, edgeEnds, tolerance)
%HASSELFINTERSECTION Test every nonadjacent edge pair at once.
%
% A simple polygon's edges meet only at shared vertices of neighbouring
% edges, so any contact between two nonadjacent edges makes it invalid. Each
% pair is one row: segment ab (first edge) against segment cd (second edge).
% The sign of the cross product (b-a) x (c-a) says on which side of line ab
% the point c lies. Two segments cross properly when c, d lie strictly on
% opposite sides of ab and a, b strictly on opposite sides of cd. They touch
% when an endpoint is on the other segment's line (cross ~ 0) and inside its
% bounding box.

% Pairs (i, j) with j >= i + 2, minus the closing pair (1, n), which are
% neighbours around the loop.
numEdges = size(edgeStarts, 1);
[firstIndex, secondIndex] = find(triu(true(numEdges), 2));
isClosingPair = firstIndex == 1 & secondIndex == numEdges;
firstIndex(isClosingPair) = [];
secondIndex(isClosingPair) = [];
a = edgeStarts(firstIndex, :);
b = edgeEnds(firstIndex, :);
c = edgeStarts(secondIndex, :);
d = edgeEnds(secondIndex, :);

% Side of each endpoint relative to the other segment's line. The tolerance
% scales with segment length because the cross product does.
firstDirection = b - a;
secondDirection = d - c;
firstLengths = hypot(firstDirection(:, 1), firstDirection(:, 2));
secondLengths = hypot(secondDirection(:, 1), secondDirection(:, 2));
scale = max(max(firstLengths, secondLengths), 1);
crossTolerance = tolerance * scale;
firstC = cross2(firstDirection, c - a);
firstD = cross2(firstDirection, d - a);
secondA = cross2(secondDirection, a - c);
secondB = cross2(secondDirection, b - c);

% Proper crossings, or an endpoint touching the other segment.
cdStraddlesAb = (firstC > crossTolerance & firstD < -crossTolerance) ...
    | (firstC < -crossTolerance & firstD > crossTolerance);
abStraddlesCd = (secondA > crossTolerance & secondB < -crossTolerance) ...
    | (secondA < -crossTolerance & secondB > crossTolerance);
isProperCrossing = cdStraddlesAb & abStraddlesCd;
isTouching = (abs(firstC) <= crossTolerance & isOnSegment(c, a, b, tolerance)) ...
    | (abs(firstD) <= crossTolerance & isOnSegment(d, a, b, tolerance)) ...
    | (abs(secondA) <= crossTolerance & isOnSegment(a, c, d, tolerance)) ...
    | (abs(secondB) <= crossTolerance & isOnSegment(b, c, d, tolerance));
intersects = any(isProperCrossing | isTouching);
end

function value = cross2(first, second)
%CROSS2 Row-wise 2-D cross product first x second.
value = first(:, 1) .* second(:, 2) - first(:, 2) .* second(:, 1);
end

function onSegment = isOnSegment(point, startPoint, endPoint, tolerance)
%ISONSEGMENT Row-wise bounding-box test for a point already on the segment's line.
onSegment = point(:, 1) >= min(startPoint(:, 1), endPoint(:, 1)) - tolerance ...
    & point(:, 1) <= max(startPoint(:, 1), endPoint(:, 1)) + tolerance ...
    & point(:, 2) >= min(startPoint(:, 2), endPoint(:, 2)) - tolerance ...
    & point(:, 2) <= max(startPoint(:, 2), endPoint(:, 2)) + tolerance;
end
