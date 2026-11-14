function [predictedMean, hasReturn, label] = resampleOntoBeams(angles, pairs, maxRange, params)
%RESAMPLEONTOBEAMS Assign each fixed beam from the old pairs bracketing it (eq. 2).
%
%   [predictedMean, hasReturn, label] = estimation.transport.resampleOntoBeams( ...
%       angles, pairs, maxRange, params)
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
% "overlap" instead of "blend" (still at the nearest value).
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
for i = 1:numBeams
    theta = angles(i);
    isBracketing = pairs.Lower <= theta & theta <= pairs.Upper;

    inSurface = pairs.IsSameSurface & isBracketing;
    if any(inSurface)
        weight = (pairs.PhiB(inSurface) - theta) ...
            ./ (pairs.PhiB(inSurface) - pairs.PhiA(inSurface));
        candidates = weight .* pairs.RhoA(inSurface) ...
            + (1 - weight) .* pairs.RhoB(inSurface);
        nearest = min(candidates);
        predictedMean(i) = nearest;
        hasReturn(i) = true;
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
