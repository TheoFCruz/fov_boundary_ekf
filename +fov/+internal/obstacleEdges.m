function [edgeStarts, edgeEnds, edgeObstacleId] = obstacleEdges(obstacles)
%OBSTACLEEDGES Stack the boundary segments of all polygon obstacles.
%
%   [edgeStarts, edgeEnds, edgeObstacleId] = fov.internal.obstacleEdges(obstacles)
%
% Edge i runs from edgeStarts(i,:) to edgeEnds(i,:) and belongs to obstacle
% edgeObstacleId(i). Edges are stacked in obstacle order, each polygon closed
% from its last vertex back to its first. Callers construct obstacles through
% the validated public FOV boundary.

edgeStarts = zeros(0, 2);
edgeEnds = zeros(0, 2);
edgeObstacleId = zeros(0, 1);
for obstacleIndex = 1:numel(obstacles)
    vertices = obstacles(obstacleIndex).Vertices;
    edgeStarts = [edgeStarts; vertices]; %#ok<AGROW>
    edgeEnds = [edgeEnds; vertices([2:end, 1], :)]; %#ok<AGROW>
    edgeObstacleId = [edgeObstacleId; repmat(obstacleIndex, size(vertices, 1), 1)]; %#ok<AGROW>
end
end
