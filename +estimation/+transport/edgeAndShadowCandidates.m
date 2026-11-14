function [predictedMean, hasReturn, label, shadowRange] = edgeAndShadowCandidates( ...
        angles, pairs, endpoints, oldMean, hasReturnOld, deltaP, deltaPsi, ...
        predictedMean, hasReturn, label, maxRange, params)
%EDGEANDSHADOWCANDIDATES Resolve "gap" beams into edge or disocclusion candidates.
%
%   [predictedMean, hasReturn, label, shadowRange] = ...
%       estimation.transport.edgeAndShadowCandidates(angles, pairs, ...
%       endpoints, oldMean, hasReturnOld, deltaP, deltaPsi, predictedMean, ...
%       hasReturn, label, maxRange, params)
%
% Beams labelled "gap" by resampleOntoBeams are bracketed only by jump pairs
% (methodology sec. 1.2, "Arestas" and "Oclusões novas"/"Desoclusões",
% eqs. 8-10; decisions D1-D2, D4). Every "gap" beam is resolved here into
% "edge", "disoccluded" or, if no jump pair gives a usable candidate, "new"
% (left for fillUncoveredBeams); the internal "gap" label never survives
% past this stage.
%
% First, decision D4: a beam whose angle equals an old endpoint's transported
% bearing exactly (|theta_i - phi_j| <= 1e-12) takes that endpoint's own
% value directly (its transported range and HasReturn, MaxRange/no-return if
% the endpoint had no return), labelled "edge". This keeps an isolated
% single-ray surface (hit by one beam, jump pairs on both sides) exact when
% the beams do not move. It runs before the corner construction below, which
% only sees beams strictly between a jump pair's two transported bearings.
%
% For each jump pair (n, f) (decision D2: for two returns, n is the smaller
% OLD mean; for a mixed pair the return side is always n), the corner is
% estimated once at the old mid-beam angle theta* and the old near range
% mu_n (eq. 8): c = R(-dpsi) * (mu_n * u(theta*) - dp), d = u(theta* - dpsi).
% Each unresolved beam strictly between the pair's transported bearings is
% intersected with the shadow edge c + t*d (eq. 10): t < 0 keeps the near
% surface (candidate rho_n, "edge"); t >= 0 enters the old shadow
% (candidate s_i clamped to [0, maxRange], "disoccluded", temporarily the
% conservative free-space limit until estimation.transport fills r_far in a
% later stage). A beam nearly parallel to d (|u_i x d| < 1e-9) gets no
% candidate from that pair. Decision D1: surface and open pairs are resolved
% before this stage runs, so only jump pairs compete here, and the smallest
% candidate range wins when more than one jump pair brackets the same beam.
% shadowRange records s_i for disoccluded beams (NaN elsewhere).

numBeams = numel(angles);
shadowRange = nan(numBeams, 1);
isGap = label == "gap";

% Decision D4: an exact transported-bearing match takes that endpoint's own
% value, ahead of the corner construction below.
gapIndices = find(isGap);
for n = 1:numel(gapIndices)
    i = gapIndices(n);
    matchIndex = find(abs(endpoints.Phi - angles(i)) <= 1e-12, 1);
    if isempty(matchIndex)
        continue;
    end
    if hasReturnOld(matchIndex)
        predictedMean(i) = endpoints.Rho(matchIndex);
        hasReturn(i) = true;
    else
        predictedMean(i) = maxRange;
        hasReturn(i) = false;
    end
    label(i) = "edge";
    isGap(i) = false;
end

% Per jump pair: corner construction, then the beams it strictly brackets.
bestValue = nan(numBeams, 1);
bestType = strings(numBeams, 1);
bestShadow = nan(numBeams, 1);
jumpPairIndices = find(pairs.IsJumpPair);
for p = 1:numel(jumpPairIndices)
    j = jumpPairIndices(p);
    [thetaNear, muNear, rhoNear] = nearPairSide(angles, oldMean, pairs, j);
    thetaFar = angles(j) + angles(j + 1) - thetaNear;
    thetaStar = (thetaNear + thetaFar) / 2;

    rotation = [cos(-deltaPsi), -sin(-deltaPsi); sin(-deltaPsi), cos(-deltaPsi)];
    corner = (muNear * [cos(thetaStar), sin(thetaStar)] - deltaP.') * rotation.';
    shadowDirection = [cos(thetaStar - deltaPsi), sin(thetaStar - deltaPsi)];

    candidateBeams = find(isGap & angles > pairs.Lower(j) & angles < pairs.Upper(j));
    for k = 1:numel(candidateBeams)
        i = candidateBeams(k);
        beamDirection = [cos(angles(i)), sin(angles(i))];
        denominator = cross2(beamDirection, shadowDirection);
        if abs(denominator) < 1e-9
            continue;
        end
        crossingParameter = cross2(corner, beamDirection) / denominator;
        if crossingParameter < 0
            candidateValue = rhoNear;
            candidateType = "edge";
            candidateShadow = NaN;
        else
            shadowDistance = cross2(corner, shadowDirection) / denominator;
            candidateValue = min(max(shadowDistance, 0), maxRange);
            candidateType = "disoccluded";
            candidateShadow = candidateValue;
        end
        if isnan(bestValue(i)) || candidateValue < bestValue(i)
            bestValue(i) = candidateValue;
            bestType(i) = candidateType;
            bestShadow(i) = candidateShadow;
        end
    end
end

resolved = isGap & ~isnan(bestValue);
predictedMean(resolved) = bestValue(resolved);
hasReturn(resolved) = true;
label(resolved) = bestType(resolved);
shadowRange(resolved) = bestShadow(resolved);

% No jump pair gave a usable candidate (parallel beam, or none bracket it):
% fall back to newly explored, filled by fillUncoveredBeams.
stillGap = isGap & ~resolved;
predictedMean(stillGap) = NaN;
hasReturn(stillGap) = false;
label(stillGap) = "new";
end

function [thetaNear, muNear, rhoNear] = nearPairSide(angles, oldMean, pairs, j)
%NEARPAIRSIDE Pick the jump pair's near side n (decision D2).

if pairs.ReturnA(j) && pairs.ReturnB(j)
    isANear = oldMean(j) <= oldMean(j + 1);
else
    isANear = pairs.ReturnA(j);
end
if isANear
    thetaNear = angles(j);
    muNear = oldMean(j);
    rhoNear = pairs.RhoA(j);
else
    thetaNear = angles(j + 1);
    muNear = oldMean(j + 1);
    rhoNear = pairs.RhoB(j);
end
end

function z = cross2(a, b)
%CROSS2 Scalar z-component of the 2-D cross product a x b.

z = a(1) * b(2) - a(2) * b(1);
end
