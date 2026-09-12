function [boundary, edgeStarts, edgeEnds] = normalizeBoundary(boundary, tolerance)
%NORMALIZEBOUNDARY Validate and close a simple polygon boundary.

if ~(isnumeric(boundary) && isreal(boundary) && ismatrix(boundary) && ...
        size(boundary, 2) == 2 && all(isfinite(boundary(:))))
    error('metrics:internal:normalizeBoundary:InvalidBoundary', ...
        'boundary must be a finite real M-by-2 numeric array.');
end

if ~(isnumeric(tolerance) && isscalar(tolerance) && isreal(tolerance) && ...
        isfinite(tolerance) && tolerance > 0)
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

if size(vertices, 1) > 1 && ...
        norm(vertices(1, :) - vertices(end, :)) <= tolerance
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
signedAreaTwice = sum(edgeStarts(:, 1) .* edgeEnds(:, 2) - ...
    edgeStarts(:, 2) .* edgeEnds(:, 1));
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
numEdges = size(edgeStarts, 1);
intersects = false;

% Nonadjacent edge crossings make the constructed boundary invalid.
for firstIndex = 1:numEdges - 1
    for secondIndex = firstIndex + 1:numEdges
        if secondIndex == firstIndex + 1 || ...
                (firstIndex == 1 && secondIndex == numEdges)
            continue;
        end

        if segmentsIntersect(edgeStarts(firstIndex, :), edgeEnds(firstIndex, :), ...
                edgeStarts(secondIndex, :), edgeEnds(secondIndex, :), tolerance)
            intersects = true;
            return;
        end
    end
end
end

function intersects = segmentsIntersect(a, b, c, d, tolerance)
firstDirection = b - a;
secondDirection = d - c;
scale = max([norm(firstDirection), norm(secondDirection), 1]);
crossTolerance = tolerance * scale;

firstC = cross2(firstDirection, c - a);
firstD = cross2(firstDirection, d - a);
secondA = cross2(secondDirection, a - c);
secondB = cross2(secondDirection, b - c);

intersects = ((firstC > crossTolerance && firstD < -crossTolerance) || ...
    (firstC < -crossTolerance && firstD > crossTolerance)) && ...
    ((secondA > crossTolerance && secondB < -crossTolerance) || ...
    (secondA < -crossTolerance && secondB > crossTolerance));

if intersects
    return;
end

intersects = (abs(firstC) <= crossTolerance && isOnSegment(c, a, b, tolerance)) || ...
    (abs(firstD) <= crossTolerance && isOnSegment(d, a, b, tolerance)) || ...
    (abs(secondA) <= crossTolerance && isOnSegment(a, c, d, tolerance)) || ...
    (abs(secondB) <= crossTolerance && isOnSegment(b, c, d, tolerance));
end

function value = cross2(first, second)
value = first(1) * second(2) - first(2) * second(1);
end

function onSegment = isOnSegment(point, startPoint, endPoint, tolerance)
onSegment = point(1) >= min(startPoint(1), endPoint(1)) - tolerance && ...
    point(1) <= max(startPoint(1), endPoint(1)) + tolerance && ...
    point(2) >= min(startPoint(2), endPoint(2)) - tolerance && ...
    point(2) <= max(startPoint(2), endPoint(2)) + tolerance;
end
