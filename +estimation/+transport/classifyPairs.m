function pairs = classifyPairs(endpoints, hasReturn, params)
%CLASSIFYPAIRS Decide which consecutive old beams share a surface (eq. 7).
%
%   pairs = estimation.transport.classifyPairs(endpoints, hasReturn, params)
%
% Pair j joins old beams j and j+1. Two returns are the same surface iff
%
%     |rho_a - rho_b| <= max(tau, rel * min(rho_a, rho_b), 3 * sqrt(var_a + var_b)),
%
% with tau = params.JumpThreshold and rel = params.JumpRelative; the
% uncertainty term keeps noise from splitting a surface into false edges.
% Two no-return beams form an open (no-return) pair. A mixed return /
% no-return pair is always a jump. Lower/Upper are the pair's transported
% bearings in ascending order, so a pair flipped by parallax still brackets.

phiA = endpoints.Phi(1:end-1);
phiB = endpoints.Phi(2:end);
rhoA = endpoints.Rho(1:end-1);
rhoB = endpoints.Rho(2:end);
varianceA = endpoints.RangeVariance(1:end-1);
varianceB = endpoints.RangeVariance(2:end);
returnA = hasReturn(1:end-1);
returnB = hasReturn(2:end);
distinctBearings = phiB ~= phiA;

jumpBound = max( ...
    max(params.JumpThreshold, params.JumpRelative .* min(rhoA, rhoB)), ...
    3 * sqrt(varianceA + varianceB));

pairs = struct();
pairs.PhiA = phiA;
pairs.PhiB = phiB;
pairs.RhoA = rhoA;
pairs.RhoB = rhoB;
pairs.Lower = min(phiA, phiB);
pairs.Upper = max(phiA, phiB);
pairs.IsSameSurface = returnA & returnB & distinctBearings ...
    & abs(rhoA - rhoB) <= jumpBound;
pairs.IsNoReturnPair = ~returnA & ~returnB & distinctBearings;
pairs.IsAnyPair = distinctBearings;
end
