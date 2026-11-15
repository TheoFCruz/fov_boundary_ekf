function [predictedMean, farRange] = applyFreeSpaceHypothesis( ...
        angles, pairs, endpoints, hasReturnOld, nearIndex, farIndex, ...
        shadowRange, label, predictedMean, maxRange, params)
%APPLYFREESPACEHYPOTHESIS Fill disoccluded beams with the eq. 11 free-space blend.
%
%   [predictedMean, farRange] = estimation.transport.applyFreeSpaceHypothesis( ...
%       angles, pairs, endpoints, hasReturnOld, nearIndex, farIndex, ...
%       shadowRange, label, predictedMean, maxRange, params)
%
% edgeAndShadowCandidates leaves each "disoccluded" beam at the conservative
% seen-free limit s_i (methodology sec. 1.2, "Desoclusões", eq. 11). That
% estimate alone cancels the CBF gradient, because the seen-free set is
% fixed in the world: it does not move with the observer. Instead, blend s_i
% toward the background range r_far,i by a constant free-space fraction eta,
%
%   mu_bar_i = s_i + eta * (r_far,i - s_i),
%
% with eta = params.FreeSpaceFraction (eta = 0 recovers the conservative
% limit, the gradient source (iv) ablation; eta = 1 assumes nothing lies
% between the corner and the background). r_far,i is the background range
% along beam i, found by extending the far surface: the intersection of the
% beam with the line through the transported far point f and its same-side
% neighbour f' (the same-surface pair (f, f') decision D2's winning jump
% pair identifies), or rho_f if f has no such neighbour, or the intersection
% is invalid (near-parallel, nonpositive). A no-return f has no background
% to extend: r_far,i = maxRange. s_i is clamped to [0, r_far,i] before the
% blend; farRange records r_far,i for disoccluded beams (NaN elsewhere).

numBeams = numel(angles);
farRange = nan(numBeams, 1);
isDisoccluded = find(label == "disoccluded");
for k = 1:numel(isDisoccluded)
    i = isDisoccluded(k);
    f = farIndex(i);
    n = nearIndex(i);
    rFar = farBackgroundRange(angles(i), f, n, pairs, endpoints, hasReturnOld, maxRange);

    sClamped = min(max(shadowRange(i), 0), rFar);
    predictedMean(i) = sClamped + params.FreeSpaceFraction * (rFar - sClamped);
    farRange(i) = rFar;
end
end

function rFar = farBackgroundRange(theta, f, n, pairs, endpoints, hasReturnOld, maxRange)
%FARBACKGROUNDRANGE Extend the far surface through f and its same-side neighbour.

if ~hasReturnOld(f)
    rFar = maxRange;
    return;
end

% f' is f's neighbour away from n, along the same surface (decision D2).
fPrime = f + sign(f - n);
numBeams = numel(endpoints.Rho);
hasNeighbor = fPrime >= 1 && fPrime <= numBeams;
if hasNeighbor
    samePairIndex = min(f, fPrime);
    hasNeighbor = pairs.IsSameSurface(samePairIndex);
end

rFar = endpoints.Rho(f);
if hasNeighbor
    pointF = endpoints.Rho(f) * [cos(endpoints.Phi(f)), sin(endpoints.Phi(f))];
    pointFPrime = endpoints.Rho(fPrime) ...
        * [cos(endpoints.Phi(fPrime)), sin(endpoints.Phi(fPrime))];
    direction = pointFPrime - pointF;
    rayDirection = [cos(theta), sin(theta)];
    denominator = cross2(rayDirection, direction);
    if abs(denominator) > 1e-9
        crossingRange = cross2(pointF, direction) / denominator;
        if crossingRange > 0
            rFar = crossingRange;
        end
    end
end
rFar = min(rFar, maxRange);
end

function z = cross2(a, b)
%CROSS2 Scalar z-component of the 2-D cross product a x b.

z = a(1) * b(2) - a(2) * b(1);
end
