function [predictedMean, hasReturn, label] = resampleOntoBeams(angles, pairs, maxRange)
%RESAMPLEONTOBEAMS Assign each fixed beam from the old pairs bracketing it (eq. 2).
%
%   [predictedMean, hasReturn, label] = estimation.transport.resampleOntoBeams( ...
%       angles, pairs, maxRange)
%
% A beam bracketed by a same-surface pair takes the linear interpolation in
% angle, w * rho_a + (1 - w) * rho_b with w = (phi_b - theta) / (phi_b -
% phi_a) ("blend"); when several pairs bracket it, the nearest surface wins
% (collision rule). Otherwise a bracketing open pair keeps it at maxRange
% with no return ("no-return"). Beams bracketed only by jump pairs are
% "jump-gap" and beams no pair brackets are "new"; both are left NaN for
% fillUncoveredBeams.

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
        predictedMean(i) = min(candidates);
        hasReturn(i) = true;
        label(i) = "blend";
    elseif any(pairs.IsNoReturnPair & isBracketing)
        predictedMean(i) = maxRange;
        label(i) = "no-return";
    elseif any(pairs.IsAnyPair & isBracketing)
        label(i) = "jump-gap";
    else
        label(i) = "new";
    end
end
end
