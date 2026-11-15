function [predictedMean, hasReturn, label, fromIndex, fromWeight] = ...
        resampleOntoBeams(angles, pairs, maxRange, params)
%RESAMPLEONTOBEAMS Assign each fixed beam from the old pairs bracketing it (eq. 2).
%
%   [predictedMean, hasReturn, label, fromIndex, fromWeight] = ...
%       estimation.transport.resampleOntoBeams(angles, pairs, maxRange, params)
%
% A beam bracketed by any same-surface pair takes the linear interpolation
% in angle, w * rho_a + (1 - w) * rho_b with w = (phi_b - theta) / (phi_b -
% phi_a) (surface candidates go first, decision D1: a far-surface pair
% cannot bracket a beam inside a jump gap, so same-surface pairs are
% resolved before jump pairs are even considered). With a single bracketing
% same-surface pair the beam is "blend" at that value. With two or more, the
% nearest candidate wins (collision rule); if the farthest candidate exceeds
% the nearest by more than max(JumpThreshold, JumpRelative * nearest), a near
% surface is sliding in front of a farther one and the beam is labelled
% "overlap" instead of "blend" (still at the nearest value). fromIndex
% records the winning pair's old indices [a, b] = [j, j+1] and fromWeight
% its weight w on a (NaN off blend/overlap beams), for
% estimation.transport.propagateCovariance's eq. 5 Jacobian row.
%
% Otherwise a bracketing open (no-return) pair keeps it at maxRange with no
% return ("no-return"). A beam bracketed only by jump pairs is left NaN with
% the internal label "gap", for estimation.transport.edgeAndShadowCandidates
% to resolve into "edge" or "disoccluded" (or fall back to "new"); a beam no
% pair brackets at all is "new". Neither "gap" nor any other internal label
% ever reaches estimation.transportBoundary's output.

numBeams = numel(angles);
predictedMean = nan(numBeams, 1);
hasReturn = false(numBeams, 1);
label = strings(numBeams, 1);
fromIndex = nan(numBeams, 2);
fromWeight = nan(numBeams, 1);
for i = 1:numBeams
    theta = angles(i);
    % A tiny tolerance: at zero motion a beam sits exactly on its own old
    % bearing, and rounding in eq. 1 must not leave the sector-edge beams
    % unbracketed.
    isBracketing = pairs.Lower - 1e-12 <= theta & theta <= pairs.Upper + 1e-12;

    inSurface = pairs.IsSameSurface & isBracketing;
    if any(inSurface)
        pairIndices = find(inSurface);
        weight = (pairs.PhiB(inSurface) - theta) ...
            ./ (pairs.PhiB(inSurface) - pairs.PhiA(inSurface));
        candidates = weight .* pairs.RhoA(inSurface) ...
            + (1 - weight) .* pairs.RhoB(inSurface);
        [nearest, nearestPosition] = min(candidates);
        predictedMean(i) = nearest;
        hasReturn(i) = true;
        winner = pairIndices(nearestPosition);
        fromIndex(i, :) = [winner, winner + 1];
        fromWeight(i) = weight(nearestPosition);
        overlapBound = max(params.JumpThreshold, params.JumpRelative * nearest);
        if max(candidates) - nearest > overlapBound
            label(i) = "overlap";
        else
            label(i) = "blend";
        end
    elseif any(pairs.IsNoReturnPair & isBracketing)
        predictedMean(i) = maxRange;
        label(i) = "no-return";
    elseif any(pairs.IsJumpPair & isBracketing)
        label(i) = "gap";
    else
        label(i) = "new";
    end
end
end
