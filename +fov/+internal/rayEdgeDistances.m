function distances = rayEdgeDistances(origin, directions, edgeStarts, edgeEnds, tolerance)
%RAYEDGEDISTANCES Distance along every ray to every line segment.
%
%   distances = fov.internal.rayEdgeDistances( ...
%       origin, directions, edgeStarts, edgeEnds, tolerance)
%
% All rays start at the 1-by-2 point origin; directions is R-by-2 (nonzero,
% normalized here). edgeStarts and edgeEnds are E-by-2. The R-by-E result
% holds the Euclidean distance from the origin to where ray r meets segment
% e, or Inf when it misses. Rays run down the rows and edges across the
% columns, so every pair is solved at once by implicit expansion.
%
% Geometry. Write the ray as  origin + t*u  (t >= 0, unit u) and the segment
% as  p + s*v  (0 <= s <= 1, v = end - start). They meet when
%
%     origin + t*u = p + s*v,   i.e.   t*u - s*v = q,   q = p - origin.
%
% Taking the 2-D cross product a x b = a1*b2 - a2*b1 of both sides with v
% removes s, and with u removes t (since u x u = v x v = 0):
%
%     t = (q x v) / (u x v),      s = (q x u) / (u x v).
%
% A hit needs t >= 0 (in front of the origin) and 0 <= s <= 1 (between the
% segment ends), each within tolerance.
%
% When u x v ~ 0 the ray and segment are parallel and the pair counts as a
% miss. For convex obstacles this loses nothing: a ray running along an
% edge first meets that edge at a corner, which the neighbouring
% (non-parallel) edge also reaches at the same distance. The one exception
% is an origin lying on the edge itself, a configuration the simulation
% does not produce. Trusted internal helper: inputs are not validated.

% Unit rays as columns (R-by-1) and edge quantities as rows (1-by-E).
unitDirections = directions ./ hypot(directions(:, 1), directions(:, 2));
ux = unitDirections(:, 1);
uy = unitDirections(:, 2);
edgeVectors = edgeEnds - edgeStarts;
vx = edgeVectors(:, 1).';
vy = edgeVectors(:, 2).';
relativeStarts = edgeStarts - origin;
qx = relativeStarts(:, 1).';
qy = relativeStarts(:, 2).';

% u x v (R-by-E). The edge length scales the parallel test so long edges
% are not misjudged as parallel.
uCrossV = ux .* vy - uy .* vx;
isParallel = abs(uCrossV) <= tolerance .* max(1, hypot(vx, vy));

% Solve for t along the ray and s along the edge (see header).
t = (qx .* vy - qy .* vx) ./ uCrossV;
s = (qx .* uy - qy .* ux) ./ uCrossV;
isHit = ~isParallel & t >= -tolerance & s >= -tolerance & s <= 1 + tolerance;

distances = inf(size(uCrossV));
distances(isHit) = max(t(isHit), 0);
end
