function [edgeStarts, edgeEnds] = polygonEdges(obstacle)
%POLYGONEDGES Return the boundary segments of a polygon obstacle.
%
%   [edgeStarts, edgeEnds] = fov.internal.polygonEdges(obstacle)
%
% Callers construct obstacles through the validated public FOV boundary.

vertices = obstacle.Vertices;
nextVertex = [2:size(vertices, 1), 1];

edgeStarts = vertices;
edgeEnds = vertices(nextVertex, :);
end
