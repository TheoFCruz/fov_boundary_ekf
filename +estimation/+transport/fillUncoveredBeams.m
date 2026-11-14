function [predictedMean, hasReturn] = fillUncoveredBeams( ...
        angles, predictedMean, hasReturn, label, maxRange, params)
%FILLUNCOVEREDBEAMS Fill "new" beams from the nearest covered beam.
%
%   [predictedMean, hasReturn] = estimation.transport.fillUncoveredBeams( ...
%       angles, predictedMean, hasReturn, label, maxRange, params)
%
% Newly explored beams (methodology sec. 1.2, "Regiões recém-exploradas";
% plan sec. 4.7) have no old point: either no pair bracketed them, or the
% edge/disocclusion stage found no usable jump-pair candidate for them.
% Every other label ("blend", "no-return", "overlap", "edge", "disoccluded")
% is covered. The nearest covered beam decides: if it is open ("no-return"),
% the uncovered beam stays open (maxRange, no return), so no distant surface
% is extended across open space; otherwise the beam continues that surface:
% it is intersected with the line through the covered beam and its same-side
% neighbour, when that neighbour is itself a surface label ("blend",
% "overlap" or "edge", excluding the weaker "disoccluded" hypothesis) and
% passes the mean-only eq. 7 range test, or else it simply copies the covered
% beam's range and HasReturn. Saturates at maxRange as no return.

coveredIndices = find(label ~= "new");
fillIndices = find(label == "new");
numBeams = numel(angles);
for n = 1:numel(fillIndices)
    i = fillIndices(n);
    if isempty(coveredIndices)
        predictedMean(i) = maxRange;
        hasReturn(i) = false;
        continue;
    end
    theta = angles(i);
    [~, nearestPosition] = min(abs(angles(coveredIndices) - theta));
    k = coveredIndices(nearestPosition);

    % Next to open space the beam stays open: never extend a distant surface.
    if label(k) == "no-return"
        predictedMean(i) = maxRange;
        hasReturn(i) = false;
        continue;
    end

    % The neighbour on the covered side of k (away from the uncovered beam)
    % continues the same surface if it is itself a surface label and passes
    % the eq. 7 range test (mean only here, so without the variance term).
    if angles(k) < theta
        neighborIndex = k - 1;
    else
        neighborIndex = k + 1;
    end
    isSurfaceLabel = neighborIndex >= 1 && neighborIndex <= numBeams ...
        && any(label(neighborIndex) == ["blend", "overlap", "edge"]);
    hasNeighbor = isSurfaceLabel;
    if hasNeighbor
        rangeK = predictedMean(k);
        rangeNeighbor = predictedMean(neighborIndex);
        sameSurfaceBound = max( ...
            params.JumpThreshold, params.JumpRelative * min(rangeK, rangeNeighbor));
        hasNeighbor = abs(rangeK - rangeNeighbor) <= sameSurfaceBound;
    end

    candidateRange = predictedMean(k);
    if hasNeighbor
        pointK = predictedMean(k) * [cos(angles(k)), sin(angles(k))];
        pointNeighbor = predictedMean(neighborIndex) ...
            * [cos(angles(neighborIndex)), sin(angles(neighborIndex))];
        direction = pointNeighbor - pointK;
        rayDirection = [cos(theta), sin(theta)];
        denominator = cross2(rayDirection, direction);
        % Skip an (almost) parallel line: no useful crossing.
        if abs(denominator) > 1e-9
            crossingRange = cross2(pointK, direction) / denominator;
            if crossingRange > 0
                candidateRange = crossingRange;
            end
        end
    end

    if candidateRange >= maxRange
        predictedMean(i) = maxRange;
        hasReturn(i) = false;
    else
        predictedMean(i) = candidateRange;
        hasReturn(i) = hasReturn(k);
    end
end
end

function z = cross2(a, b)
%CROSS2 Scalar z-component of the 2-D cross product a x b.

z = a(1) * b(2) - a(2) * b(1);
end
