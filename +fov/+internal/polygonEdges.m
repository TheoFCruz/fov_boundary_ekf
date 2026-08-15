function [edgeStarts, edgeEnds] = polygonEdges(obstacle)
%POLYGONEDGES Return the boundary segments of a polygon obstacle.
%
%   [edgeStarts, edgeEnds] = fov.internal.polygonEdges(obstacle)
%
% Each output is N-by-2. Row i of edgeStarts and edgeEnds describes one
% segment, including the closing segment from the final vertex to the first.

if ~isstruct(obstacle) || ~isfield(obstacle, 'Vertices')
    error('fov:internal:polygonEdges:InvalidObstacle', ...
        'obstacle must contain a Vertices field.');
end

vertices = obstacle.Vertices;
if ~(isnumeric(vertices) && isreal(vertices) && ismatrix(vertices) && ...
        size(vertices, 2) == 2 && size(vertices, 1) >= 3 && ...
        all(isfinite(vertices(:))))
    error('fov:internal:polygonEdges:InvalidVertices', ...
        'obstacle.Vertices must be a finite real N-by-2 array with N >= 3.');
end

vertices = double(vertices);
nextVertex = [2:size(vertices, 1), 1];

edgeStarts = vertices;
edgeEnds = vertices(nextVertex, :);
end
