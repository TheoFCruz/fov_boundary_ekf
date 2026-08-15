function [distance, points, segmentParameter, isHit] = ...
        raySegmentIntersection(rayOrigin, rayDirection, ...
        segmentStarts, segmentEnds, tolerance)
%RAYSEGMENTINTERSECTION Intersect a ray with one or more line segments.
%
%   distance = fov.internal.raySegmentIntersection( ...
%       rayOrigin, rayDirection, segmentStarts, segmentEnds)
%   [distance, points, segmentParameter, isHit] = ...
%       fov.internal.raySegmentIntersection(..., tolerance)
%
% The ray is parameterized as origin + distance * direction. The direction
% is normalized internally, so distance is Euclidean distance. Inputs
% segmentStarts and segmentEnds are N-by-2 arrays. Misses have Inf distance,
% NaN points/parameters, and false isHit values. Collinear overlap is treated
% as a hit at the nearest point on the segment in front of the ray origin.

if nargin < 4 || nargin > 5
    error('fov:internal:raySegmentIntersection:InvalidArguments', ...
        'Expected four or five input arguments.');
end

if nargin < 5 || isempty(tolerance)
    tolerance = 1e-10;
end

if ~(isnumeric(tolerance) && isscalar(tolerance) && isreal(tolerance) && ...
        isfinite(tolerance) && tolerance > 0)
    error('fov:internal:raySegmentIntersection:InvalidTolerance', ...
        'tolerance must be a finite positive scalar.');
end

rayOrigin = validatePoint(rayOrigin, 'rayOrigin');
rayDirection = validatePoint(rayDirection, 'rayDirection');

if ~(isnumeric(segmentStarts) && isreal(segmentStarts) && ...
        ismatrix(segmentStarts) && size(segmentStarts, 2) == 2 && ...
        all(isfinite(segmentStarts(:))))
    error('fov:internal:raySegmentIntersection:InvalidSegments', ...
        'segmentStarts must be a finite real N-by-2 array.');
end

if ~(isnumeric(segmentEnds) && isreal(segmentEnds) && ...
        ismatrix(segmentEnds) && size(segmentEnds, 2) == 2 && ...
        all(isfinite(segmentEnds(:))))
    error('fov:internal:raySegmentIntersection:InvalidSegments', ...
        'segmentEnds must be a finite real N-by-2 array.');
end

if size(segmentStarts, 1) ~= size(segmentEnds, 1)
    error('fov:internal:raySegmentIntersection:InvalidSegments', ...
        'segmentStarts and segmentEnds must have the same number of rows.');
end

rayOrigin = double(rayOrigin);
rayDirection = double(rayDirection);
segmentStarts = double(segmentStarts);
segmentEnds = double(segmentEnds);

directionNorm = hypot(rayDirection(1), rayDirection(2));
if directionNorm == 0
    error('fov:internal:raySegmentIntersection:ZeroDirection', ...
        'rayDirection must be nonzero.');
end
unitDirection = rayDirection / directionNorm;

numSegments = size(segmentStarts, 1);
distance = inf(numSegments, 1);
points = nan(numSegments, 2);
segmentParameter = nan(numSegments, 1);
isHit = false(numSegments, 1);

if numSegments == 0
    return;
end

segmentVectors = segmentEnds - segmentStarts;
segmentLengths = hypot(segmentVectors(:, 1), segmentVectors(:, 2));
relativeStarts = segmentStarts - rayOrigin;

crossDirectionSegments = unitDirection(1) .* segmentVectors(:, 2) - ...
    unitDirection(2) .* segmentVectors(:, 1);
parallelScale = max(1, segmentLengths);
isParallel = abs(crossDirectionSegments) <= tolerance .* parallelScale;

nonParallel = ~isParallel;
if any(nonParallel)
    starts = relativeStarts(nonParallel, :);
    vectors = segmentVectors(nonParallel, :);
    denominators = crossDirectionSegments(nonParallel);

    rayDistances = (starts(:, 1) .* vectors(:, 2) - ...
        starts(:, 2) .* vectors(:, 1)) ./ denominators;
    parameters = (starts(:, 1) .* unitDirection(2) - ...
        starts(:, 2) .* unitDirection(1)) ./ denominators;

    valid = rayDistances >= -tolerance & ...
        parameters >= -tolerance & parameters <= 1 + tolerance;
    nonParallelIndices = find(nonParallel);
    hitIndices = nonParallelIndices(valid);

    distance(hitIndices) = max(rayDistances(valid), 0);
    segmentParameter(hitIndices) = min(max(parameters(valid), 0), 1);
    isHit(hitIndices) = true;
end

parallelIndices = find(isParallel);
if ~isempty(parallelIndices)
    parallelStarts = relativeStarts(parallelIndices, :);
    parallelVectors = segmentVectors(parallelIndices, :);
    parallelLengths = segmentLengths(parallelIndices);

    % A parallel segment intersects the ray only when its supporting line is
    % collinear with the ray. The scale factor keeps the test useful away from
    % the origin while tolerance remains an absolute geometric tolerance.
    collinear = abs(parallelStarts(:, 1) .* unitDirection(2) - ...
        parallelStarts(:, 2) .* unitDirection(1)) <= ...
        tolerance .* max(1, hypot(parallelStarts(:, 1), parallelStarts(:, 2)));

    if any(collinear)
        collinearIndices = parallelIndices(collinear);
        collinearStarts = parallelStarts(collinear, :);
        collinearVectors = parallelVectors(collinear, :);
        collinearLengths = parallelLengths(collinear);

        nonPoint = collinearLengths > tolerance;
        if any(nonPoint)
            starts = collinearStarts(nonPoint, :);
            vectors = collinearVectors(nonPoint, :);
            firstDistance = starts * unitDirection.';
            secondDistance = (starts + vectors) * unitDirection.';
            nearestDistance = max(min(firstDistance, secondDistance), 0);
            farthestDistance = max(firstDistance, secondDistance);
            valid = farthestDistance >= -tolerance;

            nonPointIndices = collinearIndices(nonPoint);
            hitIndices = nonPointIndices(valid);
            distance(hitIndices) = nearestDistance(valid);
            isHit(hitIndices) = true;
        end

        pointSegments = ~nonPoint;
        if any(pointSegments)
            starts = collinearStarts(pointSegments, :);
            pointDistances = starts * unitDirection.';
            valid = pointDistances >= -tolerance;

            pointIndices = collinearIndices(pointSegments);
            hitIndices = pointIndices(valid);
            distance(hitIndices) = max(pointDistances(valid), 0);
            isHit(hitIndices) = true;
        end
    end
end

hitIndices = find(isHit);
if isempty(hitIndices)
    return;
end

points(hitIndices, :) = rayOrigin + ...
    distance(hitIndices) .* unitDirection;

% Calculate the segment parameter from the final intersection point so that
% collinear and non-collinear cases use the same convention.
hitVectors = segmentVectors(hitIndices, :);
hitLengthsSquared = sum(hitVectors.^2, 2);
nonzeroHitSegments = hitLengthsSquared > 0;
hitStarts = segmentStarts(hitIndices, :);

hitParameters = zeros(numel(hitIndices), 1);
if any(nonzeroHitSegments)
    pointOffsets = points(hitIndices(nonzeroHitSegments), :) - ...
        hitStarts(nonzeroHitSegments, :);
    vectors = hitVectors(nonzeroHitSegments, :);
    hitParameters(nonzeroHitSegments) = sum(pointOffsets .* vectors, 2) ./ ...
        hitLengthsSquared(nonzeroHitSegments);
end
segmentParameter(hitIndices) = min(max(hitParameters, 0), 1);
end

function point = validatePoint(point, inputName)
if ~(isnumeric(point) && isreal(point) && numel(point) == 2 && ...
        all(isfinite(point(:))))
    error('fov:internal:raySegmentIntersection:InvalidPoint', ...
        '%s must contain two finite real numeric values.', inputName);
end
point = reshape(point, 1, 2);
end
