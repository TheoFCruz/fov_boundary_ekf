function [predictedMean, hasReturn] = fillUncoveredBeams( ...
        angles, endpoints, pairs, hasReturnOld, predictedMean, hasReturn, label, maxRange)
%FILLUNCOVEREDBEAMS Fill "new" beams from the nearest transported old point.
%
%   [predictedMean, hasReturn] = estimation.transport.fillUncoveredBeams( ...
%       angles, endpoints, pairs, hasReturnOld, predictedMean, hasReturn, ...
%       label, maxRange)
%
% Newly explored beams (methodology sec. 1.2, "Regiões recém-exploradas";
% plan sec. 4.7) have no old point around them: either no old pair brackets
% them (they entered through a sector side) or the edge/disocclusion stage
% found no usable candidate. Each is decided by the old point next to it,
% the transported old endpoint nearest in bearing, i.e. what the old scan
% saw at that edge:
%
%   - an open (no-return) old point: the beam stays open (maxRange, no
%     return), so no surface is extended across open space;
%   - a return with a same-surface old neighbour (eq. 7 pair): that surface
%     continues, the beam is intersected with the line through the two
%     transported points;
%   - an isolated return (a surface seen by one ray): the beam inherits its
%     transported range.
%
% A result at or beyond maxRange is no return. Using the old points rather
% than the already resampled beams keeps an isolated surface at the sector
% edge (e.g. one ray on a box) from being dropped when the motion moves it
% between new beams.

numOld = numel(endpoints.Rho);
fillIndices = find(label == "new");
for n = 1:numel(fillIndices)
    i = fillIndices(n);
    theta = angles(i);
    [~, k] = min(abs(endpoints.Phi - theta));

    if ~hasReturnOld(k)
        predictedMean(i) = maxRange;
        hasReturn(i) = false;
        continue;
    end

    % Same-surface old neighbour of k on the side away from the new beam.
    neighbor = NaN;
    if k > 1 && pairs.IsSameSurface(k - 1)
        neighbor = k - 1;
    end
    if k < numOld && pairs.IsSameSurface(k) ...
            && (isnan(neighbor) ...
                || abs(endpoints.Phi(k + 1) - theta) > abs(endpoints.Phi(neighbor) - theta))
        neighbor = k + 1;
    end

    candidateRange = endpoints.Rho(k);
    if ~isnan(neighbor)
        pointK = endpoints.Rho(k) * [cos(endpoints.Phi(k)), sin(endpoints.Phi(k))];
        pointNeighbor = endpoints.Rho(neighbor) ...
            * [cos(endpoints.Phi(neighbor)), sin(endpoints.Phi(neighbor))];
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
        hasReturn(i) = true;
    end
end
end

function z = cross2(a, b)
%CROSS2 Scalar z-component of the 2-D cross product a x b.

z = a(1) * b(2) - a(2) * b(1);
end
