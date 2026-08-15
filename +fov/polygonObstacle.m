function obstacle = polygonObstacle(varargin)
%POLYGONOBSTACLE Create and validate a convex polygonal obstacle.
%
%   obstacle = fov.polygonObstacle(vertices)
%   obstacle = fov.polygonObstacle(name, vertices)
%
% Vertices must be ordered around the polygon boundary. A repeated final
% copy of the first vertex is accepted and removed.

if nargin == 1
    name = "Obstacle";
    vertices = varargin{1};
elseif nargin == 2
    name = varargin{1};
    vertices = varargin{2};
else
    error('fov:polygonObstacle:InvalidArguments', ...
        'Use polygonObstacle(vertices) or polygonObstacle(name, vertices).');
end

if ischar(name) && size(name, 1) == 1
    name = string(name);
elseif ~(isstring(name) && isscalar(name))
    error('fov:polygonObstacle:InvalidName', ...
        'Name must be a character vector or scalar string.');
end

if strlength(name) == 0
    error('fov:polygonObstacle:InvalidName', ...
        'Name must not be empty.');
end

if ~(isnumeric(vertices) && isreal(vertices) && ismatrix(vertices) && ...
        size(vertices, 2) == 2 && all(isfinite(vertices(:))))
    error('fov:polygonObstacle:InvalidVertices', ...
        'Vertices must be a finite real numeric N-by-2 array.');
end

vertices = double(vertices);

if size(vertices, 1) >= 2 && isequal(vertices(1, :), vertices(end, :))
    vertices(end, :) = [];
end

if size(vertices, 1) < 3
    error('fov:polygonObstacle:TooFewVertices', ...
        'A polygon obstacle requires at least three vertices.');
end

if any(all(diff(vertices, 1, 1) == 0, 2)) || ...
        size(unique(vertices, 'rows'), 1) ~= size(vertices, 1)
    error('fov:polygonObstacle:DuplicateVertices', ...
        'Polygon vertices must be unique and non-adjacent duplicates are not allowed.');
end

nextVertex = [2:size(vertices, 1), 1];
edges = vertices(nextVertex, :) - vertices;
nextEdges = edges(nextVertex, :);
turns = edges(:, 1) .* nextEdges(:, 2) - ...
    edges(:, 2) .* nextEdges(:, 1);

edgeScale = max(abs(edges(:)));
tolerance = 1e-12 * max(edgeScale^2, eps);

relativeVertices = vertices - vertices(1, :);
relativeNext = relativeVertices(nextVertex, :);
doubleArea = sum(relativeVertices(:, 1) .* relativeNext(:, 2) - ...
    relativeNext(:, 1) .* relativeVertices(:, 2));

if abs(doubleArea) <= tolerance
    error('fov:polygonObstacle:DegeneratePolygon', ...
        'Polygon vertices must enclose a nonzero area.');
end

nonCollinearTurns = turns(abs(turns) > tolerance);
if isempty(nonCollinearTurns) || ...
        any(sign(nonCollinearTurns) ~= sign(nonCollinearTurns(1)))
    error('fov:polygonObstacle:NonConvexPolygon', ...
        'Vertices must describe a convex polygon in boundary order.');
end

obstacle = struct('Name', name, 'Vertices', vertices);
end
